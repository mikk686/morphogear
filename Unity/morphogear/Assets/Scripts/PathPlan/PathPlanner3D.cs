// PathPlanner3D.cs — Aerial-Ground Vehicle path planning module (Unity 6.6+)
//
// Replaces: Pathfinding3D.cs, Path3DRequestManager.cs, Heap.cs, IHeapItem.cs.
//
// A* formulation is unchanged from the published version:
//   g(n') = g(n) + Octile3D(n,n') + penalty(n') + HeightPenalty(n,n')
//   h(n)  = Octile3D(n, goal)    [17 diagonal / 10 straight, integer]
//   tie-break on lower h at equal f
//   post-processing: collinear simplification, optional Bezier smoothing
//
// TWO ROBUSTNESS FIXES over the original, both required because the agent
// reserves the cells it stands in:
//   1. Every traversability test is OWNER-AWARE. The querying agent's own
//      reservation never blocks it, so standing still cannot make you unplannable.
//   2. If start or goal is still unusable (buried in geometry, boxed in by another
//      robot), a breadth-first escape finds the nearest usable cell instead of
//      failing. The original only looked one ring out and gave up.
//
// UNITY 6.6 MIGRATION NOTE: the owner handle is an EntityId (not an int). The
// "no owner" value is EntityId.None; PathGrid3D treats any !IsValid() owner as
// anonymous, so anonymous searches see every reservation as blocking.

using System;
using System.Collections.Generic;
using UnityEngine;

namespace AgvPlanning
{
    public enum HeightPenaltyAxis
    {
        /// <summary>Original published behaviour: reacts to the plan-Y (world Z) axis.</summary>
        LegacyPlanY = 0,

        /// <summary>Reacts to the altitude axis (cell.z) — actual climb/descent.</summary>
        Altitude = 1,
    }

    public readonly struct PathResult
    {
        public readonly Vector3[] Waypoints;
        public readonly bool Success;
        public readonly int ExpandedNodes;
        public readonly string Failure;

        public PathResult(Vector3[] waypoints, bool success, int expanded, string failure = null)
        {
            Waypoints = waypoints;
            Success = success;
            ExpandedNodes = expanded;
            Failure = failure;
        }

        public static PathResult Failed(string reason, int expanded = 0) =>
            new(Array.Empty<Vector3>(), false, expanded, reason);
    }

    [RequireComponent(typeof(PathGrid3D))]
    [DisallowMultipleComponent]
    public sealed class PathPlanner3D : MonoBehaviour
    {
        [Header("Cost model")]
        public int heightChangePenalty = 50;
        public int levelMovePenalty = 5;
        public HeightPenaltyAxis heightPenaltyAxis = HeightPenaltyAxis.LegacyPlanY;

        [Header("Post-processing")]
        public bool simplify = true;
        public bool smooth;
        [Min(1f)]
        public float smoothness = 1f;

        [Header("Robustness")]
        [Tooltip("Cells to search outward when start/goal is unusable. 0 disables the escape.")]
        public int escapeSearchLimit = 512;
        [Tooltip("Log why a search failed.")]
        public bool logFailures = true;

        [Header("Limits")]
        [Tooltip("Abort after this many expansions. 0 = unlimited.")]
        public int maxExpansions = 0;

        public static PathPlanner3D Instance { get; private set; }

        PathGrid3D _grid;
        int[] _gCost, _hCost, _parent, _stamp;
        int _searchStamp;
        IndexHeap _open;
        HashSet<int> _closed;
        readonly Queue<int> _escapeQueue = new();
        readonly HashSet<int> _escapeSeen = new();

        readonly Queue<Request> _queue = new();
        bool _busy;

        readonly struct Request
        {
            public readonly Vector3 Start, End;
            public readonly EntityId Owner;
            public readonly Action<PathResult> Callback;

            public Request(Vector3 start, Vector3 end, EntityId owner, Action<PathResult> callback)
            {
                Start = start;
                End = end;
                Owner = owner;
                Callback = callback;
            }
        }

        void Awake()
        {
            Instance = this;
            _grid = GetComponent<PathGrid3D>();
        }

        void OnDestroy()
        {
            if (Instance == this) Instance = null;
        }

        // ---- public API -------------------------------------------------------

        /// <summary>Drop-in for the old Path3DRequestManager.RequestPath (no owner context).</summary>
        public static void RequestPath(Vector3 start, Vector3 end, Action<Vector3[], bool> callback)
        {
            if (Instance == null)
            {
                Debug.LogError($"{nameof(PathPlanner3D)}: no instance in the scene.");
                callback?.Invoke(Array.Empty<Vector3>(), false);
                return;
            }

            Instance.Enqueue(start, end, EntityId.None, r => callback?.Invoke(r.Waypoints, r.Success));
        }

        /// <summary>
        /// Queue a search. Pass the agent's GetEntityId() as <paramref name="owner"/> so its
        /// own reserved footprint is ignored — this is what unblocks a stationary robot.
        /// Pass EntityId.None for an anonymous search.
        /// </summary>
        public void Enqueue(Vector3 start, Vector3 end, EntityId owner, Action<PathResult> callback)
        {
            _queue.Enqueue(new Request(start, end, owner, callback));
            if (!_busy) _ = PumpAsync();
        }

        /// <summary>
        /// Awaitable search. <paramref name="owner"/> defaults to default(EntityId), which
        /// PathGrid3D treats exactly like EntityId.None (EntityId.None is not a compile-time
        /// constant, so it cannot be used as the default argument itself).
        /// </summary>
        public async Awaitable<PathResult> FindPathAsync(Vector3 start, Vector3 end, EntityId owner = default)
        {
            while (_busy) await Awaitable.NextFrameAsync(destroyCancellationToken);
            _busy = true;
            try
            {
                return await RunAsync(start, end, owner);
            }
            finally
            {
                _busy = false;
            }
        }

        async Awaitable PumpAsync()
        {
            _busy = true;
            try
            {
                while (_queue.Count > 0)
                {
                    var request = _queue.Dequeue();
                    var result = await RunAsync(request.Start, request.End, request.Owner);
                    request.Callback?.Invoke(result);
                }
            }
            finally
            {
                _busy = false;
            }
        }

        async Awaitable<PathResult> RunAsync(Vector3 startPos, Vector3 targetPos, EntityId owner)
        {
            var ct = destroyCancellationToken;
            while (!_grid.IsBuilt || _grid.IsBuilding) await Awaitable.NextFrameAsync(ct);

            EnsureCapacity(_grid.Count);
            var startIndex = _grid.IndexFromWorld(startPos);
            var targetIndex = _grid.IndexFromWorld(targetPos);

            _grid.BeginRead();
            PathResult result;
            try
            {
                await Awaitable.BackgroundThreadAsync();
                result = Search(startIndex, targetIndex, owner);
            }
            finally
            {
                await Awaitable.MainThreadAsync();
                _grid.EndRead();
            }

            if (!result.Success && logFailures && result.Failure != null)
                Debug.LogWarning($"[{nameof(PathPlanner3D)}] {result.Failure}");

            return result;
        }

        void EnsureCapacity(int count)
        {
            if (_gCost != null && _gCost.Length >= count) return;
            _gCost = new int[count];
            _hCost = new int[count];
            _parent = new int[count];
            _stamp = new int[count];
            _searchStamp = 0;
            _open = new IndexHeap(count);
            _closed = new HashSet<int>(1024);
        }

        // ---- A* ---------------------------------------------------------------

        PathResult Search(int startIndex, int targetIndex, EntityId owner)
        {
            // Owner-aware: the caller's own reservation is transparent to it.
            if (!_grid.IsTraversableFor(startIndex, owner))
            {
                startIndex = FindNearestUsable(startIndex, owner);
                if (startIndex < 0) return PathResult.Failed("start is enclosed, no usable cell nearby");
            }

            if (!_grid.IsTraversableFor(targetIndex, owner))
            {
                targetIndex = FindNearestUsable(targetIndex, owner);
                if (targetIndex < 0) return PathResult.Failed("goal is enclosed, no usable cell nearby");
            }

            if (startIndex == targetIndex)
                return new PathResult(new[] { _grid.NodeAt(targetIndex).WorldPosition }, true, 0);

            _searchStamp++;
            _open.Clear();
            _closed.Clear();

            Touch(startIndex);
            _gCost[startIndex] = 0;
            _hCost[startIndex] = Octile(startIndex, targetIndex);
            _parent[startIndex] = -1;
            _open.Push(startIndex, _hCost[startIndex], _hCost[startIndex]);

            var expanded = 0;
            var found = false;
            Span<int> neighbours = stackalloc int[PathGrid3D.MaxNeighbours];

            while (_open.Count > 0)
            {
                var current = _open.Pop();
                if (!_closed.Add(current)) continue;
                expanded++;

                if (current == targetIndex)
                {
                    found = true;
                    break;
                }
                if (maxExpansions > 0 && expanded >= maxExpansions)
                    return PathResult.Failed($"expansion limit {maxExpansions} reached", expanded);

                var n = _grid.GetNeighbours(current, neighbours);
                for (var i = 0; i < n; i++)
                {
                    var next = neighbours[i];
                    if (!_grid.IsTraversableFor(next, owner) || _closed.Contains(next)) continue;

                    var cost = _gCost[current] + Octile(current, next) + _grid.NodeAt(next).MovementPenalty + HeightPenalty(current, next);

                    if (_stamp[next] == _searchStamp && cost >= _gCost[next]) continue;

                    Touch(next);
                    _gCost[next] = cost;
                    _hCost[next] = Octile(next, targetIndex);
                    _parent[next] = current;
                    _open.Push(next, cost + _hCost[next], _hCost[next]);
                }
            }

            if (!found) return PathResult.Failed("no route (goal unreachable)", expanded);

            var waypoints = Retrace(startIndex, targetIndex);
            if (smooth) waypoints = Bezier(waypoints, smoothness);
            return new PathResult(waypoints, true, expanded);
        }

        void Touch(int index)
        {
            if (_stamp[index] == _searchStamp) return;
            _stamp[index] = _searchStamp;
            _gCost[index] = int.MaxValue;
            _hCost[index] = 0;
            _parent[index] = -1;
        }

        /// <summary>
        /// Breadth-first outward search for the closest cell usable by this owner.
        /// Replaces the original single-ring scan, which failed as soon as the agent was
        /// surrounded by its own reservation or by another robot.
        /// </summary>
        int FindNearestUsable(int from, EntityId owner)
        {
            if (escapeSearchLimit <= 0) return -1;

            _escapeQueue.Clear();
            _escapeSeen.Clear();
            _escapeQueue.Enqueue(from);
            _escapeSeen.Add(from);

            Span<int> neighbours = stackalloc int[26];
            var visited = 0;

            while (_escapeQueue.Count > 0 && visited < escapeSearchLimit)
            {
                var current = _escapeQueue.Dequeue();
                visited++;
                if (current != from && _grid.IsTraversableFor(current, owner)) return current;

                var n = _grid.GetNeighbours26(current, neighbours);
                for (var i = 0; i < n; i++)
                {
                    var next = neighbours[i];
                    if (_escapeSeen.Add(next)) _escapeQueue.Enqueue(next);
                }
            }

            return -1;
        }

        // ---- cost -------------------------------------------------------------

        int HeightPenalty(int fromIndex, int toIndex)
        {
            var a = _grid.CoordsOf(fromIndex);
            var b = _grid.CoordsOf(toIndex);
            var delta = heightPenaltyAxis == HeightPenaltyAxis.Altitude
                            ? a.z - b.z
                            : a.y - b.y; // original published behaviour
            return delta != 0 ? heightChangePenalty : levelMovePenalty;
        }

        /// <summary>3D octile distance. 17 per diagonal step, 10 per axis step (original weights).</summary>
        int Octile(int aIndex, int bIndex)
        {
            var a = _grid.CoordsOf(aIndex);
            var b = _grid.CoordsOf(bIndex);
            var dx = Mathf.Abs(a.x - b.x);
            var dy = Mathf.Abs(a.y - b.y);
            var dz = Mathf.Abs(a.z - b.z);
            var min = Mathf.Min(dx, Mathf.Min(dy, dz));
            var max = Mathf.Max(dx, Mathf.Max(dy, dz));
            var mid = dx + dy + dz - min - max;
            return 17 * min + 10 * (mid - min) + 10 * (max - min);
        }

        // ---- post-processing --------------------------------------------------

        Vector3[] Retrace(int startIndex, int targetIndex)
        {
            // target -> start, excluding start (matches the original RetracePath)
            var chain = new List<int>(64);
            var current = targetIndex;
            var guard = _grid.Count + 1;

            while (current != startIndex && current >= 0 && guard-- > 0)
            {
                chain.Add(current);
                current = _parent[current];
            }

            var waypoints = simplify ? Simplify(chain) : Convert(chain);
            Array.Reverse(waypoints);
            return waypoints;
        }

        Vector3[] Convert(List<int> chain)
        {
            var result = new Vector3[chain.Count];
            for (var i = 0; i < chain.Count; i++) result[i] = _grid.NodeAt(chain[i]).WorldPosition;
            return result;
        }

        /// <summary>Drops collinear intermediate cells. Same rule as the original SimplifyPath.</summary>
        Vector3[] Simplify(List<int> chain)
        {
            if (chain.Count == 0) return Array.Empty<Vector3>();

            var waypoints = new List<Vector3>(chain.Count) { _grid.NodeAt(chain[0]).WorldPosition };
            var previousDirection = Vector3Int.zero;

            for (var i = 0; i < chain.Count - 1; i++)
            {
                var direction = _grid.CoordsOf(chain[i + 1]) - _grid.CoordsOf(chain[i]);
                if (direction != previousDirection)
                    waypoints.Add(_grid.NodeAt(chain[i]).WorldPosition);
                previousDirection = direction;
            }

            return waypoints.ToArray();
        }

        /// <summary>De Casteljau interpolation over the whole waypoint set (original behaviour).</summary>
        static Vector3[] Bezier(Vector3[] waypoints, float smoothness)
        {
            if (waypoints.Length <= 2) return waypoints;
            if (smoothness < 1f) smoothness = 1f;

            var segments = waypoints.Length * Mathf.RoundToInt(smoothness) - 1;
            var result = new Vector3[segments + 1];
            var scratch = new Vector3[waypoints.Length];

            for (var step = 0; step <= segments; step++)
            {
                var t = Mathf.InverseLerp(0, segments, step);
                Array.Copy(waypoints, scratch, waypoints.Length);

                for (var j = waypoints.Length - 1; j > 0; j--)
                    for (var i = 0; i < j; i++)
                        scratch[i] = (1f - t) * scratch[i] + t * scratch[i + 1];

                result[step] = scratch[0];
            }

            return result;
        }

        /// <summary>
        /// Binary min-heap over flat node indices, ordered by f then h. Allows duplicate
        /// entries (lazy deletion) — the closed set filters stale pops.
        /// </summary>
        sealed class IndexHeap
        {
            int[] _index, _f, _h;
            int _count;

            public IndexHeap(int capacity)
            {
                capacity = Mathf.Max(64, capacity);
                _index = new int[capacity];
                _f = new int[capacity];
                _h = new int[capacity];
            }

            public int Count => _count;
            public void Clear() => _count = 0;

            public void Push(int index, int f, int h)
            {
                if (_count == _index.Length) Grow();

                var i = _count++;
                _index[i] = index;
                _f[i] = f;
                _h[i] = h;

                while (i > 0)
                {
                    var parent = (i - 1) >> 1;
                    if (!Less(i, parent)) break;
                    Swap(i, parent);
                    i = parent;
                }
            }

            public int Pop()
            {
                var result = _index[0];
                _count--;

                if (_count > 0)
                {
                    _index[0] = _index[_count];
                    _f[0] = _f[_count];
                    _h[0] = _h[_count];
                    var i = 0;
                    while (true)
                    {
                        var left = 2 * i + 1;
                        var right = left + 1;
                        var best = i;
                        if (left < _count && Less(left, best)) best = left;
                        if (right < _count && Less(right, best)) best = right;
                        if (best == i) break;
                        Swap(i, best);
                        i = best;
                    }
                }

                return result;
            }

            bool Less(int a, int b) => _f[a] != _f[b] ? _f[a] < _f[b] : _h[a] < _h[b];

            void Swap(int a, int b)
            {
                (_index[a], _index[b]) = (_index[b], _index[a]);
                (_f[a], _f[b]) = (_f[b], _f[a]);
                (_h[a], _h[b]) = (_h[b], _h[a]);
            }

            void Grow()
            {
                var size = _index.Length * 2;
                Array.Resize(ref _index, size);
                Array.Resize(ref _f, size);
                Array.Resize(ref _h, size);
            }
        }
    }
}
