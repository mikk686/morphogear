// PathRouteRenderer.cs — Aerial-Ground Vehicle path planning module (Unity 6.x)
//
// THE FIX for "the path is not visible in the Build".
//
// OnDrawGizmos is editor-only: Unity strips it from player builds entirely, so any
// visualisation built on Gizmos exists only while the Editor is drawing the Scene
// or Game view with gizmos enabled. This component draws the same information with
// real renderers — LineRenderer for the polyline, GPU-instanced meshes for the
// waypoint markers — so it shows up in a standalone build and in screen recordings.
//
// Attach next to AgvPathAgent. Nothing to wire: it reads the agent every frame.

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace AgvPlanning
{
    [RequireComponent(typeof(AgvPathAgent))]
    [DisallowMultipleComponent]
    public sealed class PathRouteRenderer : MonoBehaviour
    {
        [System.Serializable]
        public struct LineStyle
        {
            public bool show;
            public Color color;
            [Min(0.001f)] public float width;
            public bool showMarkers;
            [Min(0.001f)] public float markerSize;
        }

        [Header("Material")]
        [Tooltip("Unlit material for the lines. Leave empty to auto-create one.")]
        public Material lineMaterial;
        [Tooltip("Mesh for waypoint markers. Leave empty to auto-create a cube.")]
        public Mesh markerMesh;
        [Tooltip("GPU-instanced material for markers. Leave empty to reuse lineMaterial.")]
        public Material markerMaterial;

        [Header("Active path (agent -> target)")]
        public LineStyle activePath = new()
        {
            show = true, color = Color.black, width = 0.05f, showMarkers = true, markerSize = 0.12f,
        };

        [Header("Committed route")]
        public LineStyle route = new()
        {
            show = true, color = Color.green, width = 0.06f, showMarkers = true, markerSize = 0.14f,
        };

        [Header("Pending leg")]
        public LineStyle pendingLeg = new()
        {
            show = true, color = Color.red, width = 0.04f, showMarkers = true, markerSize = 0.1f,
        };

        [Header("Placement")]
        [Tooltip("Lift lines by this much so they don't z-fight with the ground.")]
        public float verticalOffset = 0.05f;
        [Tooltip("Prepend the agent's current position to the active path.")]
        public bool anchorToAgent = true;

        AgvPathAgent _agent;
        LineRenderer _activeLine, _routeLine, _pendingLine;
        readonly List<Vector3> _scratch = new();
        readonly Matrix4x4[] _batch = new Matrix4x4[511];

        // one bucket per style so each keeps its own colour
        readonly List<Matrix4x4> _activeMarkers = new();
        readonly List<Matrix4x4> _routeMarkers = new();
        readonly List<Matrix4x4> _pendingMarkers = new();
        MaterialPropertyBlock _activeBlock, _routeBlock, _pendingBlock;

        static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        static readonly int ColorId = Shader.PropertyToID("_Color");

        void Awake()
        {
            _agent = GetComponent<AgvPathAgent>();
            EnsureAssets();

            _activeLine = CreateLine("ActivePathLine", activePath);
            _routeLine = CreateLine("RouteLine", route);
            _pendingLine = CreateLine("PendingLegLine", pendingLeg);

            _activeBlock = new MaterialPropertyBlock();
            _routeBlock = new MaterialPropertyBlock();
            _pendingBlock = new MaterialPropertyBlock();
        }

        void EnsureAssets()
        {
            if (lineMaterial == null)
            {
                // URP first, then the built-in pipeline fallback
                var shader = Shader.Find("Universal Render Pipeline/Unlit") ??
                             Shader.Find("Sprites/Default");
                lineMaterial = new Material(shader) { name = "AgvPathLine (auto)" };
                lineMaterial.enableInstancing = true;
            }

            if (markerMaterial == null) markerMaterial = lineMaterial;

            if (markerMesh == null)
            {
                var temp = GameObject.CreatePrimitive(PrimitiveType.Cube);
                markerMesh = temp.GetComponent<MeshFilter>().sharedMesh;
                Destroy(temp);
            }
        }

        LineRenderer CreateLine(string childName, LineStyle style)
        {
            var host = new GameObject(childName);
            host.transform.SetParent(transform, false);

            var line = host.AddComponent<LineRenderer>();
            line.useWorldSpace = true;
            line.material = lineMaterial;
            line.widthMultiplier = style.width;
            line.numCapVertices = 2;
            line.numCornerVertices = 2;
            line.shadowCastingMode = ShadowCastingMode.Off;
            line.receiveShadows = false;
            line.alignment = LineAlignment.View;
            line.textureMode = LineTextureMode.Stretch;
            line.startColor = line.endColor = style.color;
            line.positionCount = 0;
            return line;
        }

        void LateUpdate()
        {
            if (_agent == null) return;

            _activeMarkers.Clear();
            _routeMarkers.Clear();
            _pendingMarkers.Clear();

            _scratch.Clear();
            if (anchorToAgent) _scratch.Add(transform.position);
            _scratch.AddRange(_agent.ActivePath);
            Apply(_activeLine, activePath, _scratch, anchorToAgent ? 1 : 0, _activeMarkers);

            Apply(_routeLine, route, _agent.Route, 0, _routeMarkers);

            _scratch.Clear();
            if (_agent.Route.Count > 0) _scratch.Add(_agent.Route[^1]);
            _scratch.AddRange(_agent.PendingLeg);
            Apply(_pendingLine, pendingLeg, _scratch, _agent.Route.Count > 0 ? 1 : 0, _pendingMarkers);

            DrawMarkers(_activeMarkers, activePath.color, _activeBlock);
            DrawMarkers(_routeMarkers, route.color, _routeBlock);
            DrawMarkers(_pendingMarkers, pendingLeg.color, _pendingBlock);
        }

        void Apply(LineRenderer line, LineStyle style, IReadOnlyList<Vector3> points,
                   int markerStart, List<Matrix4x4> markers)
        {
            if (line == null) return;

            if (!style.show)
            {
                line.positionCount = 0;
                return;
            }

            line.startColor = line.endColor = style.color;
            line.widthMultiplier = style.width;

            var lift = Vector3.up * verticalOffset;
            if (points.Count < 2)
            {
                line.positionCount = 0;
            }
            else
            {
                line.positionCount = points.Count;
                for (var i = 0; i < points.Count; i++) line.SetPosition(i, points[i] + lift);
            }

            if (!style.showMarkers) return;
            var scale = Vector3.one * style.markerSize;
            for (var i = markerStart; i < points.Count; i++)
                markers.Add(Matrix4x4.TRS(points[i] + lift, Quaternion.identity, scale));
        }

        void DrawMarkers(List<Matrix4x4> markers, Color color, MaterialPropertyBlock block)
        {
            if (markers.Count == 0 || markerMesh == null || markerMaterial == null) return;

            block.SetColor(BaseColorId, color); // URP
            block.SetColor(ColorId, color);     // built-in
            var rp = new RenderParams(markerMaterial)
            {
                matProps = block,
                shadowCastingMode = ShadowCastingMode.Off,
                receiveShadows = false,
                worldBounds = new Bounds(transform.position, Vector3.one * 10000f),
            };

            for (var offset = 0; offset < markers.Count; offset += _batch.Length)
            {
                var length = Mathf.Min(_batch.Length, markers.Count - offset);
                markers.CopyTo(offset, _batch, 0, length);
                Graphics.RenderMeshInstanced(rp, markerMesh, 0, _batch, length);
            }
        }
    }
}
