// DynamicObstacle.cs — Aerial-Ground Vehicle path planning module (Unity 6.x)
//
// THE FIX for "the grid does not update when objects move".
//
// Put this on every collider that can move (other robots, doors, crates, lifts).
// It watches its own bounds and, when they shift past a threshold, hands the grid
// BOTH the bounds it left and the bounds it now occupies. Re-probing the union is
// what makes the vacated cells become Passable again — probing only the new
// position would leave a permanent obstacle scar in the old one.
//
// GridDirtyQueue (bottom of this file) coalesces every request in a frame into a
// single physics batch, so 30 movers cost one rebuild, not 30.

using System.Collections.Generic;
using UnityEngine;

namespace AgvPlanning
{
    [DisallowMultipleComponent]
    public sealed class DynamicObstacle : MonoBehaviour
    {
        [Tooltip("Grid to notify. Auto-found if left empty.")]
        public PathGrid3D grid;

        [Tooltip("Re-probe once the collider has moved this far (world units). " +
                 "Set near nodeRadius — smaller values just burn physics queries.")]
        public float moveThreshold = 0.5f;

        [Tooltip("Extra margin added to the dirty box, in cells. 1 covers the danger dilation ring.")]
        public int paddingCells = 1;

        [Tooltip("How often to test for movement. 0 = every frame.")]
        public float checkInterval = 0.1f;

        Collider[] _colliders;
        Bounds _lastBounds;
        bool _hasLast;
        float _nextCheck;

        void Awake()
        {
            if (grid == null) grid = FindFirstObjectByType<PathGrid3D>();
            _colliders = GetComponentsInChildren<Collider>();
        }

        void OnEnable()
        {
            _hasLast = false;
            _nextCheck = 0f;
        }

        void OnDisable()
        {
            // vacate: the cells we occupied must go back to Passable
            if (_hasLast && grid != null) GridDirtyQueue.Enqueue(grid, Pad(_lastBounds));
            _hasLast = false;
        }

        void LateUpdate()
        {
            if (grid == null || !grid.IsBuilt) return;
            if (checkInterval > 0f)
            {
                if (Time.time < _nextCheck) return;
                _nextCheck = Time.time + checkInterval;
            }

            if (!TryGetBounds(out var current)) return;

            if (!_hasLast)
            {
                _lastBounds = current;
                _hasLast = true;
                GridDirtyQueue.Enqueue(grid, Pad(current));
                return;
            }

            var moved = (current.center - _lastBounds.center).sqrMagnitude >
                        moveThreshold * moveThreshold;
            var resized = (current.size - _lastBounds.size).sqrMagnitude >
                          moveThreshold * moveThreshold;
            if (!moved && !resized) return;

            // both boxes: vacated cells AND newly occupied cells
            GridDirtyQueue.Enqueue(grid, Pad(_lastBounds));
            GridDirtyQueue.Enqueue(grid, Pad(current));
            _lastBounds = current;
        }

        bool TryGetBounds(out Bounds bounds)
        {
            bounds = default;
            var found = false;
            foreach (var c in _colliders)
            {
                if (c == null || !c.enabled) continue;
                if (!found) { bounds = c.bounds; found = true; }
                else bounds.Encapsulate(c.bounds);
            }

            return found;
        }

        Bounds Pad(Bounds b)
        {
            var margin = grid.NodeDiameter * Mathf.Max(0, paddingCells) +
                         grid.nodeRadius * grid.checkRadiusModifier;
            b.Expand(margin * 2f);
            return b;
        }

        /// <summary>Force an immediate re-probe of this obstacle's current footprint.</summary>
        public void MarkDirty()
        {
            if (grid == null || !TryGetBounds(out var b)) return;
            GridDirtyQueue.Enqueue(grid, Pad(b));
        }
    }

    /// <summary>
    /// Per-grid coalescing queue. Every dirty box submitted during a frame is merged
    /// and re-probed in one batched job at end of frame.
    /// </summary>
    [DefaultExecutionOrder(1000)]
    public sealed class GridDirtyQueue : MonoBehaviour
    {
        static readonly Dictionary<PathGrid3D, GridDirtyQueue> Queues = new();

        PathGrid3D _grid;
        readonly List<Bounds> _boxes = new();
        bool _running;

        public static void Enqueue(PathGrid3D grid, Bounds worldBounds)
        {
            if (grid == null) return;
            if (!Queues.TryGetValue(grid, out var queue) || queue == null)
            {
                queue = grid.gameObject.AddComponent<GridDirtyQueue>();
                queue._grid = grid;
                Queues[grid] = queue;
            }

            queue._boxes.Add(worldBounds);
        }

        void OnDestroy()
        {
            if (_grid != null) Queues.Remove(_grid);
        }

        async void LateUpdate()
        {
            if (_running || _boxes.Count == 0 || _grid == null) return;
            if (!_grid.IsBuilt || _grid.IsBuilding) return;

            _running = true;
            try
            {
                var batch = new List<Bounds>(_boxes);
                _boxes.Clear();
                await _grid.RebuildAsync(batch);
            }
            finally
            {
                _running = false;
            }
        }
    }
}
