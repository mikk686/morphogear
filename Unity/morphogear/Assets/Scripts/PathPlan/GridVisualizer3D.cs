// GridVisualizer3D.cs — Aerial-Ground Vehicle path planning module (Unity 6.x)
//
// Replaces: Grid3DRepresentation.cs, GridColor.cs and the per-node GameObject
// hierarchy under "Grid3D". Nothing is instantiated — the grid is drawn with
// GPU-instanced batches, so 200k cells cost a few draw calls instead of 200k
// Transforms. Works in a player build (unlike gizmos).
//
// Reservation state comes from PathGrid3D.ReservedBy, not from Walkable, so
// "occupied by a robot" and "blocked by geometry" are shown as different colours.
//
// Material requirement: GPU instancing enabled, per-instance _BaseColor (URP/Lit)
// or _Color (built-in).

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace AgvPlanning
{
    [RequireComponent(typeof(PathGrid3D))]
    [DisallowMultipleComponent]
    public sealed class GridVisualizer3D : MonoBehaviour
    {
        const int BatchSize = 511; // RenderMeshInstanced tolerates 1023; 511 is a safe margin

        enum Bucket { Passable = 0, Dangerous = 1, Impassable = 2, Reserved = 3 }

        [Header("Rendering")]
        public bool display = true;
        public Mesh nodeMesh;
        [Tooltip("Must have GPU instancing enabled.")]
        public Material nodeMaterial;
        [Range(0.02f, 1f)] public float nodeScale = 0.1f;
        public ShadowCastingMode shadows = ShadowCastingMode.Off;

        [Header("Filter")]
        [Tooltip("Draw only this altitude layer (cell.z). -1 draws every layer.")]
        public int layerFilter = -1;
        [Tooltip("Skip free cells. Strongly recommended on large grids.")]
        public bool hidePassable = true;
        public bool drawDangerous = true;
        public bool drawImpassable = true;
        public bool drawReserved = true;

        [Header("Colors")]
        public Color passableColor = new(1f, 1f, 1f, 0.10f);
        public Color dangerousColor = new(1f, 0.85f, 0f, 0.35f);
        public Color impassableColor = new(1f, 0.1f, 0.1f, 0.55f);
        public Color reservedColor = new(0f, 0.5f, 1f, 0.55f);

        [Header("Performance")]
        [Tooltip("Minimum seconds between bucket rebuilds. Node changes are coalesced.")]
        public float rebuildInterval = 0.1f;

        PathGrid3D _grid;
        static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        static readonly int ColorId = Shader.PropertyToID("_Color");

        readonly List<Matrix4x4>[] _buckets = new List<Matrix4x4>[4];
        readonly MaterialPropertyBlock[] _blocks = new MaterialPropertyBlock[4];
        readonly Matrix4x4[] _batch = new Matrix4x4[BatchSize];
        bool _dirty = true;
        float _nextRebuild;

        void Awake()
        {
            _grid = GetComponent<PathGrid3D>();
            for (var i = 0; i < _buckets.Length; i++)
            {
                _buckets[i] = new List<Matrix4x4>(1024);
                _blocks[i] = new MaterialPropertyBlock();
            }
        }

        void OnEnable()
        {
            _grid.Rebuilt += MarkDirty;
            _grid.NodeChanged += OnNodeChanged;
            _dirty = true;
        }

        void OnDisable()
        {
            _grid.Rebuilt -= MarkDirty;
            _grid.NodeChanged -= OnNodeChanged;
        }

        void MarkDirty() => _dirty = true;
        void OnNodeChanged(int _) => _dirty = true;

        void LateUpdate()
        {
            if (!display || nodeMesh == null || nodeMaterial == null || !_grid.IsBuilt) return;

            if (_dirty && Time.time >= _nextRebuild)
            {
                Rebuild();
                _nextRebuild = Time.time + Mathf.Max(0f, rebuildInterval);
            }

            var bounds = _grid.WorldBounds;
            if (!hidePassable) Draw(Bucket.Passable, passableColor, bounds);
            if (drawDangerous) Draw(Bucket.Dangerous, dangerousColor, bounds);
            if (drawImpassable) Draw(Bucket.Impassable, impassableColor, bounds);
            if (drawReserved) Draw(Bucket.Reserved, reservedColor, bounds);
        }

        void Draw(Bucket bucket, Color color, Bounds bounds)
        {
            var matrices = _buckets[(int)bucket];
            if (matrices.Count == 0) return;

            var block = _blocks[(int)bucket];
            block.SetColor(BaseColorId, color); // URP
            block.SetColor(ColorId, color);     // built-in

            var rp = new RenderParams(nodeMaterial)
            {
                matProps = block,
                shadowCastingMode = shadows,
                receiveShadows = false,
                worldBounds = bounds,
            };

            for (var offset = 0; offset < matrices.Count; offset += BatchSize)
            {
                var length = Mathf.Min(BatchSize, matrices.Count - offset);
                matrices.CopyTo(offset, _batch, 0, length);
                Graphics.RenderMeshInstanced(rp, nodeMesh, 0, _batch, length);
            }
        }

        void Rebuild()
        {
            _dirty = false;
            foreach (var bucket in _buckets) bucket.Clear();

            var scale = Vector3.one * (_grid.NodeDiameter * nodeScale);
            var rotation = Quaternion.identity;

            for (var index = 0; index < _grid.Count; index++)
            {
                if (layerFilter >= 0 && _grid.CoordsOf(index).z != layerFilter) continue;

                ref var node = ref _grid.NodeAt(index);

                // reservation wins visually: it's the state you care about while debugging
                Bucket bucket;
                if (_grid.ReservedBy(index) != EntityId.None) bucket = Bucket.Reserved;
                else if (node.Walkable == Walkable.Impassable) bucket = Bucket.Impassable;
                else if (node.Walkable == Walkable.Dangerous) bucket = Bucket.Dangerous;
                else
                {
                    if (hidePassable) continue;
                    bucket = Bucket.Passable;
                }

                _buckets[(int)bucket].Add(Matrix4x4.TRS(node.WorldPosition, rotation, scale));
            }
        }
    }
}
