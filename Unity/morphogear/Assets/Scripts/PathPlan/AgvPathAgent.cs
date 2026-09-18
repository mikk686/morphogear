// AgvPathAgent.cs — Aerial-Ground Vehicle path planning module (Unity 6.6+)
//
// Replaces: UNIT3DMorphoGear.cs (planning half; mission glue is AgvMissionBuilder).
//
// Occupancy model, which is where the old version wedged itself:
//   * the agent reserves its cell (+ optionally the 26-neighbourhood) via
//     PathGrid3D.Reserve, using GetEntityId() as the owner handle;
//   * reservations live OUTSIDE Walkable, so the planner is told to ignore this
//  agent's own cells (owner-aware) and a grid rebuild cannot wipe them;
//   * therefore a stationary robot never blocks its own start cell, and other
//     robots still see it as occupied.
//
// UNITY 6.6 MIGRATION NOTE: GetInstanceID() (int) has been replaced by
// GetEntityId() (EntityId) everywhere. EntityId cannot be cast to int any more.
//
// Visualisation is split out: gizmos here for the Editor, PathRouteRenderer for
// anything that must survive into a player build.

using System.Collections.Generic;
using UnityEngine;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace AgvPlanning
{
    [DisallowMultipleComponent]
    public class AgvPathAgent : MonoBehaviour
    {
        [Header("References")]
        public PathGrid3D grid;
        public PathPlanner3D planner;
        public Transform target;

        [Header("Replanning")]
        [Tooltip("Replan when the target moves further than this (world units).")]
        public float targetMoveThreshold = 0.25f;
        [Tooltip("Replan when the agent itself moves this far. 0 disables.")]
        public float selfMoveReplanDistance = 0f;
        [Tooltip("Minimum seconds between automatic replans.")]
        public float replanCooldown = 0.25f;
        [Tooltip("Replan whenever the grid finishes a rebuild (dynamic obstacles moved).")]
        public bool replanOnGridChange = true;

        [Header("Occupancy")]
        [Tooltip("Reserve this agent's cell so other agents avoid it.")]
        public bool reserveOwnCell = true;
        [Tooltip("Also reserve the 26-neighbourhood while stationary. Off keeps corridors usable.")]
        public bool reserveNeighbourhood = false;
        [Tooltip("Reserve the neighbourhood only while not moving (original behaviour).")]
        public bool neighbourhoodOnlyWhenParked = true;

        [Header("Startup")]
        public bool rebuildGridOnStart = true;

        [Header("Input")]
        public bool enableKeyboardShortcuts = true;

        [Header("Editor gizmos")]
        [Tooltip("Editor-only. For builds use PathRouteRenderer.")]
        public bool drawGizmos = true;

        public Vector3[] ActivePath { get; private set; } = System.Array.Empty<Vector3>();
        public List<Vector3> Route { get; } = new();
        public Vector3[] PendingLeg { get; private set; } = System.Array.Empty<Vector3>();

        public bool IsMoving { get; set; }
        public int PathFoundCount { get; private set; }
        public int CellsMoved { get; private set; }
        public Vector3Int CurrentCell { get; private set; }
        public string LastFailure { get; private set; }

        /// <summary>Owner handle for grid reservations and owner-aware planning (Unity 6.6 EntityId).</summary>
        public EntityId Owner => GetEntityId();

        Vector3 _lastTargetPosition, _lastSelfPosition;
        int _lastCellIndex = -1;
        float _nextReplanTime;
        bool _replanQueued;
        readonly List<int> _reserved = new(PathGrid3D.MaxNeighbours + 1);
        readonly int[] _neighbourScratch = new int[26];

        protected virtual void Awake()
        {
            if (grid == null) grid = FindFirstObjectByType<PathGrid3D>();
            if (planner == null) planner = FindFirstObjectByType<PathPlanner3D>();
        }

        protected virtual void OnEnable()
        {
            if (grid != null && replanOnGridChange) grid.Rebuilt += OnGridRebuilt;
        }

        protected virtual void OnDisable()
        {
            if (grid == null) return;
            if (replanOnGridChange) grid.Rebuilt -= OnGridRebuilt;
            grid.ClearOwner(Owner);
        }

        protected virtual async void Start()
        {
            _lastSelfPosition = transform.position;
            if (target != null) _lastTargetPosition = target.position;
            if (Route.Count == 0) Route.Add(transform.position);

            if (rebuildGridOnStart && grid != null) await grid.BuildAsync();
            UpdateOccupancy();
            RequestPaths();
        }

        protected virtual void Update()
        {
            if (grid == null || !grid.IsBuilt) return;

            if (Route.Count == 0) Route.Add(transform.position);
            Route[0] = transform.position;

            if (enableKeyboardShortcuts) PollShortcuts();

            UpdateOccupancy();

            if (target == null) return;

            var targetMoved = (target.position - _lastTargetPosition).sqrMagnitude >
                              targetMoveThreshold * targetMoveThreshold;
            var selfMoved = selfMoveReplanDistance > 0f &&
                            (transform.position - _lastSelfPosition).sqrMagnitude >
                                selfMoveReplanDistance * selfMoveReplanDistance;

            if (targetMoved)
            {
                _lastTargetPosition = target.position;
                IsMoving = true;
            }
            if (selfMoved) _lastSelfPosition = transform.position;

            if ((targetMoved || selfMoved || _replanQueued) && Time.time >= _nextReplanTime)
            {
                _replanQueued = false;
                _nextReplanTime = Time.time + Mathf.Max(0f, replanCooldown);
                RequestPaths();
            }
        }

        void OnGridRebuilt() => _replanQueued = true;

        void PollShortcuts()
        {
#if ENABLE_INPUT_SYSTEM
            var keyboard = Keyboard.current;
            if (keyboard == null) return;
            if (keyboard.pageUpKey.wasReleasedThisFrame) CommitPendingLeg();
            if (keyboard.pageDownKey.wasReleasedThisFrame) ClearRoute();
#else
            if (Input.GetKeyUp(KeyCode.PageUp)) CommitPendingLeg();
            if (Input.GetKeyUp(KeyCode.PageDown)) ClearRoute();
#endif
        }

        // ---- occupancy --------------------------------------------------------

        /// <summary>
        /// Refreshes this agent's reservation set. Cheap: only re-reserves when the agent
        /// changed cell. Never writes Walkable, so it cannot corrupt the physics state.
        /// </summary>
        public void UpdateOccupancy()
        {
            if (grid == null || !grid.IsBuilt) return;

            var index = grid.IndexFromWorld(transform.position);
            if (index == _lastCellIndex) return;

            if (_lastCellIndex >= 0) CellsMoved++;
            _lastCellIndex = index;
            CurrentCell = grid.CoordsOf(index);

            if (!reserveOwnCell)
            {
                grid.ClearOwner(Owner);
                return;
            }

            _reserved.Clear();
            _reserved.Add(index);

            var wantNeighbours = reserveNeighbourhood &&
                                 (!neighbourhoodOnlyWhenParked || !IsMoving);
            if (wantNeighbours)
            {
                var n = grid.GetNeighbours26(index, _neighbourScratch);
                for (var i = 0; i < n; i++) _reserved.Add(_neighbourScratch[i]);
            }

            grid.Reserve(Owner, _reserved);
        }

        // ---- planning ---------------------------------------------------------

        public void RequestPaths()
        {
            if (planner == null || target == null) return;

            planner.Enqueue(transform.position, target.position, Owner, result =>
            {
                LastFailure = result.Failure;
                if (!result.Success) return;
                ActivePath = result.Waypoints;
                PathFoundCount++;
            });

            var from = Route.Count > 0 ? Route[^1] : transform.position;
            planner.Enqueue(from, target.position, Owner, result =>
            {
                if (result.Success) PendingLeg = result.Waypoints;
            });
        }

        /// <summary>Appends the pending leg to the committed route (PageUp).</summary>
        public void CommitPendingLeg()
        {
            if (PendingLeg.Length == 0)
            {
                RequestPaths();
                return;
            }
            Route.AddRange(PendingLeg);
            PendingLeg = System.Array.Empty<Vector3>();
        }

        public void ClearRoute()
        {
            Route.Clear();
            Route.Add(transform.position);
            PendingLeg = System.Array.Empty<Vector3>();
        }

        public Awaitable RebuildGrid(Bounds worldBounds) => grid.RebuildAsync(worldBounds);

        // ---- editor gizmos ----------------------------------------------------

        void OnDrawGizmos()
        {
            if (!drawGizmos) return;

            DrawPolyline(ActivePath, Color.black, transform.position);
            DrawPolyline(Route, Color.green, null);
            DrawPolyline(PendingLeg, Color.red, Route.Count > 0 ? Route[^1] : null);
        }

        static void DrawPolyline(IReadOnlyList<Vector3> points, Color color, Vector3? anchor)
        {
            if (points == null || points.Count == 0) return;
            Gizmos.color = color;

            var previous = anchor;
            foreach (var point in points)
            {
                Gizmos.DrawCube(point, Vector3.one * 0.1f);
                if (previous.HasValue) Gizmos.DrawLine(previous.Value, point);
                previous = point;
            }
        }
    }
}
