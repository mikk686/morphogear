// AgvMissionBuilder.cs — Aerial-Ground Vehicle path planning module (Unity 6.x)
//
// Replaces: UNIT3DMorphoGear.GenerateMission / UpdateMission / TakePathFromROS.
//
// The original had two near-identical builders (one for RobotAction+ManagerNode,
// one for ActionExecution+MissionManager) that differed only in which types they
// instantiated — and the RobotAction one had an index bug where the walk-run
// consumed the point that should have started the next segment.
//
// Here the route is segmented ONCE into a backend-neutral list, and thin adapters
// translate segments into your action types. Fix a segmentation bug once, both
// backends get it.

using System.Collections.Generic;
using UnityEngine;

namespace AgvPlanning
{
    public enum MissionSegmentKind
    {
        Walk,     // ground traversal, one or more waypoints
        Takeoff,  // climb to Waypoints[0].y
        Fly,      // aerial traversal, one or more waypoints
        Land,     // descend to ground
    }

    public readonly struct MissionSegment
    {
        public readonly MissionSegmentKind Kind;
        public readonly IReadOnlyList<Vector3> Waypoints;

        public MissionSegment(MissionSegmentKind kind, IReadOnlyList<Vector3> waypoints)
        {
            Kind = kind;
            Waypoints = waypoints;
        }

        public Vector3 First => Waypoints[0];
        public Vector3 Last => Waypoints[Waypoints.Count - 1];
        public override string ToString() => $"{Kind} x{Waypoints.Count} -> {Last}";
    }

    [DisallowMultipleComponent]
    public sealed class AgvMissionBuilder : MonoBehaviour
    {
        [Header("Segmentation")]
        [Tooltip("A waypoint above this altitude counts as flight.")]
        public float minFlightHeight = 0.5f;
        [Tooltip("Emit an explicit Takeoff/Land segment at each ground<->air transition.")]
        public bool emitTransitions = true;
        public bool logSegments;

        [Header("Source")]
        public AgvPathAgent agent;

        readonly List<MissionSegment> _segments = new();
        int _consumedWaypoints;

        public IReadOnlyList<MissionSegment> Segments => _segments;

        void Awake()
        {
            if (agent == null) agent = GetComponent<AgvPathAgent>();
        }

        /// <summary>Rebuilds the whole mission from the agent's route.</summary>
        public IReadOnlyList<MissionSegment> Build()
        {
            _segments.Clear();
            _consumedWaypoints = 0;
            return Append();
        }

        /// <summary>Segments only the waypoints added since the last call — for incremental routes.</summary>
        public IReadOnlyList<MissionSegment> Append()
        {
            var route = agent != null ? agent.Route : null;
            if (route == null || route.Count <= _consumedWaypoints) return _segments;

            var slice = route.GetRange(_consumedWaypoints, route.Count - _consumedWaypoints);
            _consumedWaypoints = route.Count;

            var airborne = _segments.Count > 0 &&
                           (_segments[^1].Kind == MissionSegmentKind.Fly ||
                            _segments[^1].Kind == MissionSegmentKind.Takeoff);

            Segment(slice, ref airborne, _segments);

            if (logSegments)
                foreach (var s in _segments) Debug.Log($"[{name}] {s}");

            return _segments;
        }

        /// <summary>
        /// Splits a waypoint list into ground/air runs. Pure function — unit-testable and
        /// free of the off-by-one that the original while-loop version had.
        /// </summary>
        public void Segment(IReadOnlyList<Vector3> waypoints, ref bool airborne, List<MissionSegment> output)
        {
            var i = 0;
            while (i < waypoints.Count)
            {
                var wantsFlight = waypoints[i].y >= minFlightHeight;

                // consume the whole run that stays on the same medium
                var start = i;
                while (i < waypoints.Count && waypoints[i].y >= minFlightHeight == wantsFlight) i++;
                var run = new List<Vector3>(i - start);
                for (var k = start; k < i; k++) run.Add(waypoints[k]);

                if (wantsFlight)
                {
                    if (!airborne && emitTransitions)
                        output.Add(new MissionSegment(MissionSegmentKind.Takeoff, new[] { run[0] }));
                    output.Add(new MissionSegment(MissionSegmentKind.Fly, run));
                    airborne = true;
                }
                else
                {
                    if (airborne && emitTransitions)
                        output.Add(new MissionSegment(MissionSegmentKind.Land, new[] { run[0] }));
                    output.Add(new MissionSegment(MissionSegmentKind.Walk, run));
                    airborne = false;
                }
            }
        }

        // ---------------------------------------------------------------------
        // Adapters. These are the ONLY places that know about your action types.
        // Uncomment the one(s) your scene uses and delete the other.
        // ---------------------------------------------------------------------

#if AGV_ROBOTACTION_BACKEND
        public ManagerNode managerNode;

        /// <summary>Feeds the segmented mission into ManagerNode (RobotAction backend).</summary>
        public void PushToManagerNode()
        {
            if (managerNode == null) { Debug.LogError($"[{name}] managerNode not assigned."); return; }

            managerNode.AddAction(new RobotAction.Neutral());
            foreach (var segment in Build())
            {
                switch (segment.Kind)
                {
                    case MissionSegmentKind.Walk:
                        managerNode.AddAction(new RobotAction.Walk(new List<Vector3>(segment.Waypoints)));
                        break;
                    case MissionSegmentKind.Takeoff:
                        managerNode.AddAction(new RobotAction.Takeoff(segment.First.y));
                        break;
                    case MissionSegmentKind.Fly:
                        foreach (var p in segment.Waypoints) managerNode.AddAction(new RobotAction.Fly(p));
                        break;
                    case MissionSegmentKind.Land:
                        managerNode.AddAction(new RobotAction.Land());
                        break;
                }
            }

            Debug.Log($"[{name}] actions: {managerNode.Actions.Count}");
        }
#endif

#if AGV_ACTIONEXECUTION_BACKEND
        public MissionManager missionManager;

        /// <summary>Feeds the segmented mission into MissionManager (ActionExecution backend).</summary>
        public void PushToMissionManager()
        {
            if (missionManager == null) { Debug.LogError($"[{name}] missionManager not assigned."); return; }

            foreach (var segment in Append())
            {
                switch (segment.Kind)
                {
                    case MissionSegmentKind.Walk:
                        // ActionExecution.Walk takes a 2D destination in (z, x) order.
                        foreach (var p in segment.Waypoints)
                            missionManager.AddAction(new ActionExecution.Walk(new Vector2(p.z, p.x)));
                        break;
                    case MissionSegmentKind.Takeoff:
                        missionManager.AddAction(new ActionExecution.Takeoff(segment.First.y));
                        break;
                    case MissionSegmentKind.Fly:
                        foreach (var p in segment.Waypoints) missionManager.AddAction(new ActionExecution.Fly(p));
                        break;
                    case MissionSegmentKind.Land:
                        missionManager.AddAction(new ActionExecution.Land());
                        break;
                }
            }

            Debug.Log($"[{name}] actions: {missionManager.Actions.Count}");
        }
#endif

        public void Clear()
        {
            _segments.Clear();
            _consumedWaypoints = 0;
        }
    }
}
