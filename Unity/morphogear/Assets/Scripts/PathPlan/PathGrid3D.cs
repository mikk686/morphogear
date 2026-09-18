// PathGrid3D.cs — Aerial-Ground Vehicle path planning module (Unity 6.6+)
//
// Replaces: Grid3D.cs, Node3D.cs.
//
// Axis convention (from the original project, named explicitly):
//   cell.x -> world X   |   cell.y -> world Z   |   cell.z -> ALTITUDE (world Y)
// gridWorldSize is therefore (width X, depth Z, altitude Y).
//
// KEY DESIGN RULE: physics-derived state and agent reservations live in SEPARATE
// arrays. _solid/_nodes[].Walkable come from colliders only; _reservedBy holds an
// owner id. That makes (a) incremental rebuild idempotent — it cannot erase
// reservations, and (b) an agent able to ignore its own footprint when planning.
// Mixing the two in one enum is what wedged the planner.
//
// UNITY 6.6 MIGRATION NOTE: owner ids are EntityId end-to-end. EntityId is no
// longer convertible to int (CS0619), so every "0 = free" convention became
// EntityId.None / !IsValid(). No planner logic was changed.

using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace AgvPlanning
{
    /// <summary>Physics-derived traversability. Agent occupancy is NOT in here — see ReservedBy.</summary>
    public enum Walkable : byte
    {
        Passable = 0,
        Impassable = 1,
        Dangerous = 2, // passable, but adjacent to an obstacle
    }

    public struct GridNode
    {
        public Vector3 WorldPosition;
        public float GroundHeight;
        public int BasePenalty;     // terrain + altitude layer; survives dilation
        public int MovementPenalty; // BasePenalty + danger surcharge
        public Walkable Walkable;

        public readonly bool IsTraversable => Walkable != Walkable.Impassable;
    }

    public enum AltitudeMode
    {
        /// <summary>y = ground + baseHeightOffset + z * nodeDiameter. Layers follow terrain.</summary>
        GroundConforming = 0,

        /// <summary>y = ground for every z (exact geometry of the Unity 2022 script).</summary>
        LegacyFlat = 1,

        /// <summary>y = gridOrigin.y + z * nodeDiameter. Absolute altitude bands.</summary>
        Absolute = 2,
    }

    [DisallowMultipleComponent]
    public sealed class PathGrid3D : MonoBehaviour
    {
        public const int MaxNeighbours = 10; // 8 planar + up + down

        [Serializable]
        public struct TerrainType
        {
            public LayerMask terrainMask;
            public int terrainPenalty;
        }

        [Header("Extent")]
        [Tooltip("x = world width (X), y = world depth (Z), z = altitude range (Y).")]
        public Vector3 gridWorldSize = new(50f, 50f, 10f);
        public float nodeRadius = 0.5f;
        [Tooltip("Vertical offset of the lowest layer, e.g. half the robot height.")]
        public float baseHeightOffset = 0.19f;

        [Header("Obstacles")]
        public LayerMask unwalkableMask;
        [Tooltip("Obstacle probe radius = nodeRadius * this.")]
        public float checkRadiusModifier = 2f;
        [Tooltip("Penalty added to a node adjacent to an impassable node (26-neighbourhood).")]
        public int dangerPenalty = 25;
        [Tooltip("Mark cells adjacent to obstacles as Dangerous.")]
        public bool dilateDanger = true;

        [Header("Terrain")]
        [Tooltip("Layers used for the downward ground probe. Leave empty to use walkableRegions.")]
        public LayerMask groundMask;
        public TerrainType[] walkableRegions = Array.Empty<TerrainType>();
        public AltitudeMode altitudeMode = AltitudeMode.GroundConforming;
        public float groundProbeHeight = 50f;
        public float groundProbeDistance = 100f;

        [Header("Altitude penalties")]
        [Tooltip("Penalty per altitude layer (index = cell.z). Index 0 keeps the terrain penalty. " +
                 "Last entry repeats for higher layers. Original defaults: 0,200,100,50,25.")]
        public int[] altitudeLayerPenalty = { 0, 200, 100, 50, 25 };

        [Header("Build")]
        public bool buildOnStart = true;
        [Tooltip("Cells resolved per frame during the collider read-back. Lower = smoother.")]
        public int nodesPerFrame = 20000;

        // ---- data -------------------------------------------------------------

        GridNode[] _nodes;
        byte[] _solid;          // 1 = obstacle overlap, authoritative physics state
        EntityId[] _reservedBy; // EntityId.None = free, otherwise owner id   (was int[])
        int _sizeX, _sizeY, _sizeZ, _strideX, _strideY;
        float _nodeDiameter;
        Vector3 _origin;

        int _obstacleMask, _groundProbeMask;
        readonly Dictionary<int, int> _layerPenalty = new();            // key = layer index (still int)
        readonly Dictionary<EntityId, int> _colliderPenalty = new();    // key = collider EntityId (was int)
        readonly Dictionary<EntityId, List<int>> _reservations = new(); // key = owner EntityId   (was int)

        int _readers;
        readonly List<(int index, EntityId owner)> _deferredReservations = new();
        readonly HashSet<int> _dirty = new();

        public Vector3Int Size => new(_sizeX, _sizeY, _sizeZ);
        public int Count => _nodes?.Length ?? 0;
        public int MaxSize => Count;
        public float NodeDiameter => _nodeDiameter;
        public Vector3 Origin => _origin;
        public bool IsBuilt => _nodes != null;
        public bool IsBuilding { get; private set; }
        public bool IsLocked => _readers > 0;

        /// <summary>Flat index of every node whose state changed. Consumed by the visualizer.</summary>
        public event Action<int> NodeChanged;
        public event Action Rebuilt;

        void Awake()
        {
            RecomputeDimensions();
            _obstacleMask = unwalkableMask.value;

            _layerPenalty.Clear();
            var regionMask = 0;
            foreach (var region in walkableRegions)
            {
                regionMask |= region.terrainMask.value;
                var layer = LayerOf(region.terrainMask.value);
                if (layer >= 0) _layerPenalty[layer] = region.terrainPenalty;
            }

            _groundProbeMask = groundMask.value != 0 ? groundMask.value
                               : regionMask != 0 ? regionMask
                                                     : Physics.DefaultRaycastLayers;
        }

        async void Start()
        {
            if (buildOnStart) await BuildAsync();
        }

        void RecomputeDimensions()
        {
            _nodeDiameter = Mathf.Max(0.01f, nodeRadius * 2f);
            _sizeX = Mathf.Max(1, Mathf.RoundToInt(gridWorldSize.x / _nodeDiameter));
            _sizeY = Mathf.Max(1, Mathf.RoundToInt(gridWorldSize.y / _nodeDiameter));
            _sizeZ = Mathf.Max(1, Mathf.RoundToInt(gridWorldSize.z / _nodeDiameter));
            _strideY = _sizeZ;
            _strideX = _sizeY * _sizeZ;
            _origin = transform.position - Vector3.right * (gridWorldSize.x * 0.5f) - Vector3.forward * (gridWorldSize.y * 0.5f) + Vector3.up * baseHeightOffset;
        }

        static int LayerOf(int mask) => mask == 0 ? -1 : Mathf.RoundToInt(Mathf.Log(mask, 2f));

        // ---- indexing ---------------------------------------------------------

        public int IndexOf(int x, int y, int z) => x * _strideX + y * _strideY + z;

        public Vector3Int CoordsOf(int index)
        {
            var x = index / _strideX;
            var rest = index - x * _strideX;
            return new Vector3Int(x, rest / _strideY, rest % _strideY);
        }

        public bool InBounds(int x, int y, int z) =>
            (uint)x < (uint)_sizeX && (uint)y < (uint)_sizeY && (uint)z < (uint)_sizeZ;

        public ref GridNode NodeAt(int index) => ref _nodes[index];
        public ref GridNode NodeAt(int x, int y, int z) => ref _nodes[IndexOf(x, y, z)];

        public Vector3Int CellFromWorld(Vector3 world)
        {
            var px = Mathf.Clamp01((world.x - _origin.x) / Mathf.Max(0.0001f, gridWorldSize.x));
            var py = Mathf.Clamp01((world.z - _origin.z) / Mathf.Max(0.0001f, gridWorldSize.y));
            var pz = Mathf.Clamp01((world.y - _origin.y) / Mathf.Max(0.0001f, gridWorldSize.z));
            return new Vector3Int(
                Mathf.Min(_sizeX - 1, Mathf.RoundToInt((_sizeX - 1) * px)),
                Mathf.Min(_sizeY - 1, Mathf.RoundToInt((_sizeY - 1) * py)),
                Mathf.Min(_sizeZ - 1, (int)((_sizeZ - 1) * pz)));
        }

        public int IndexFromWorld(Vector3 world)
        {
            var c = CellFromWorld(world);
            return IndexOf(c.x, c.y, c.z);
        }

        Vector3 CellCenter(int x, int y, int z) => new(
            _origin.x + x * _nodeDiameter + nodeRadius,
            _origin.y + z * _nodeDiameter + nodeRadius,
            _origin.z + y * _nodeDiameter + nodeRadius);

        /// <summary>World-space AABB of the whole grid.</summary>
        public Bounds WorldBounds => new(
            _origin + new Vector3(gridWorldSize.x, gridWorldSize.z, gridWorldSize.y) * 0.5f,
            new Vector3(gridWorldSize.x, gridWorldSize.z, gridWorldSize.y));

        // ---- build ------------------------------------------------------------

        /// <summary>Full (re)build. Preserves existing reservations.</summary>
        public async Awaitable BuildAsync()
        {
            RecomputeDimensions();
            var count = _sizeX * _sizeY * _sizeZ;
            if (_nodes == null || _nodes.Length != count)
            {
                _nodes = new GridNode[count];
                _solid = new byte[count];
                _reservedBy = new EntityId[count];
                Array.Fill(_reservedBy, EntityId.None); // do not rely on default(EntityId) == None
            }

            await ProbeAsync(Vector3Int.zero, new Vector3Int(_sizeX - 1, _sizeY - 1, _sizeZ - 1));
            Rebuilt?.Invoke();
        }

        /// <summary>
        /// Re-probes only the cells overlapping <paramref name="worldBounds"/>. This is the
        /// dynamic-obstacle entry point — call it with the old AND new bounds of a mover.
        /// </summary>
        public async Awaitable RebuildAsync(Bounds worldBounds)
        {
            if (!IsBuilt)
            {
                await BuildAsync();
                return;
            }

            var lo = CellFromWorld(worldBounds.min);
            var hi = CellFromWorld(worldBounds.max);
            await ProbeAsync(Vector3Int.Min(lo, hi), Vector3Int.Max(lo, hi));
            Rebuilt?.Invoke();
        }

        /// <summary>Re-probes the union of several boxes in one physics batch.</summary>
        public async Awaitable RebuildAsync(IReadOnlyList<Bounds> boxes)
        {
            if (boxes == null || boxes.Count == 0) return;
            if (!IsBuilt)
            {
                await BuildAsync();
                return;
            }

            var lo = new Vector3Int(int.MaxValue, int.MaxValue, int.MaxValue);
            var hi = new Vector3Int(int.MinValue, int.MinValue, int.MinValue);
            foreach (var b in boxes)
            {
                var a = CellFromWorld(b.min);
                var c = CellFromWorld(b.max);
                lo = Vector3Int.Min(lo, Vector3Int.Min(a, c));
                hi = Vector3Int.Max(hi, Vector3Int.Max(a, c));
            }

            await ProbeAsync(lo, hi);
            Rebuilt?.Invoke();
        }

        async Awaitable ProbeAsync(Vector3Int lo, Vector3Int hi)
        {
            while (IsBuilding) await Awaitable.NextFrameAsync(destroyCancellationToken);
            IsBuilding = true;
            try
            {
                lo = Vector3Int.Max(lo, Vector3Int.zero);
                hi = Vector3Int.Min(hi, new Vector3Int(_sizeX - 1, _sizeY - 1, _sizeZ - 1));
                if (hi.x < lo.x || hi.y < lo.y || hi.z < lo.z) return;

                var spanX = hi.x - lo.x + 1;
                var spanY = hi.y - lo.y + 1;
                var spanZ = hi.z - lo.z + 1;
                var cellCount = spanX * spanY * spanZ;
                var columnCount = spanX * spanY;

                var probeRadius = nodeRadius * checkRadiusModifier;
                var qObstacle = new QueryParameters(_obstacleMask, false, QueryTriggerInteraction.Ignore, false);
                var qGround = new QueryParameters(_groundProbeMask, false, QueryTriggerInteraction.Ignore, false);

                var overlapCmds = new NativeArray<OverlapSphereCommand>(cellCount, Allocator.TempJob);
                var overlapHits = new NativeArray<ColliderHit>(cellCount, Allocator.TempJob);
                var groundCmds = new NativeArray<RaycastCommand>(columnCount, Allocator.TempJob);
                var groundHits = new NativeArray<RaycastHit>(columnCount, Allocator.TempJob);

                try
                {
                    // one downward probe per COLUMN, not per cell — sizeZ times fewer rays
                    for (var ix = 0; ix < spanX; ix++)
                        for (var iy = 0; iy < spanY; iy++)
                        {
                            var c = CellCenter(lo.x + ix, lo.y + iy, 0);
                            groundCmds[ix * spanY + iy] = new RaycastCommand(
                                new Vector3(c.x, _origin.y + groundProbeHeight, c.z),
                                Vector3.down, qGround, groundProbeDistance);
                        }

                    for (var ix = 0; ix < spanX; ix++)
                        for (var iy = 0; iy < spanY; iy++)
                            for (var iz = 0; iz < spanZ; iz++)
                            {
                                var i = (ix * spanY + iy) * spanZ + iz;
                                overlapCmds[i] = new OverlapSphereCommand(
                                    CellCenter(lo.x + ix, lo.y + iy, lo.z + iz), probeRadius, qObstacle);
                            }

                    var h1 = RaycastCommand.ScheduleBatch(groundCmds, groundHits, 32, 1);
                    var h2 = OverlapSphereCommand.ScheduleBatch(overlapCmds, overlapHits, 32, 1);
                    var handle = JobHandle.CombineDependencies(h1, h2);

                    while (!handle.IsCompleted) await Awaitable.NextFrameAsync(destroyCancellationToken);
                    handle.Complete();

                    WriteColumns(lo, spanX, spanY, groundHits);
                    await WriteCells(lo, spanX, spanY, spanZ, overlapHits);
                    await DilateAsync(lo, hi);
                }
                finally
                {
                    overlapCmds.Dispose();
                    overlapHits.Dispose();
                    groundCmds.Dispose();
                    groundHits.Dispose();
                }
            }
            finally
            {
                IsBuilding = false;
            }
        }

        // per-column ground height + terrain penalty. Main thread: resolves hit.collider.
        void WriteColumns(Vector3Int lo, int spanX, int spanY, NativeArray<RaycastHit> hits)
        {
            _colliderPenalty.Clear();
            _columnHeight ??= new float[_sizeX * _sizeY];
            _columnPenalty ??= new int[_sizeX * _sizeY];

            for (var ix = 0; ix < spanX; ix++)
                for (var iy = 0; iy < spanY; iy++)
                {
                    var hit = hits[ix * spanY + iy];
                    var column = (lo.x + ix) * _sizeY + (lo.y + iy);

                    var id = hit.colliderEntityId; // EntityId, never cast to int
                    var collider = id.IsValid() ? hit.collider : null;
                    if (collider == null)
                    {
                        _columnHeight[column] = _origin.y;
                        _columnPenalty[column] = 0;
                        continue;
                    }

                    if (!_colliderPenalty.TryGetValue(id, out var penalty))
                    {
                        _layerPenalty.TryGetValue(collider.gameObject.layer, out penalty);
                        _colliderPenalty[id] = penalty;
                    }

                    _columnHeight[column] = collider.bounds.max.y;
                    _columnPenalty[column] = penalty;
                }
        }

        float[] _columnHeight;
        int[] _columnPenalty;

        async Awaitable WriteCells(Vector3Int lo, int spanX, int spanY, int spanZ,
                                   NativeArray<ColliderHit> hits)
        {
            var budget = Mathf.Max(1024, nodesPerFrame);
            var processed = 0;

            for (var ix = 0; ix < spanX; ix++)
                for (var iy = 0; iy < spanY; iy++)
                {
                    var x = lo.x + ix;
                    var y = lo.y + iy;
                    var column = x * _sizeY + y;
                    var ground = _columnHeight[column];
                    var terrainPenalty = _columnPenalty[column];

                    // Synchronous helper: all ref-local work happens there (not allowed in async methods).
                    ProcessColumnZ(x, y, ix, iy, spanY, spanZ, ground, terrainPenalty, lo.z, hits);

                    processed += spanZ;
                    if (processed >= budget)
                    {
                        processed = 0;
                        await Awaitable.NextFrameAsync(destroyCancellationToken);
                    }
                }
        }

        // Synchronous, so `ref var node` is legal here.
        void ProcessColumnZ(int x, int y, int ix, int iy, int spanY, int spanZ,
                            float ground, int terrainPenalty, int loZ, NativeArray<ColliderHit> hits)
        {
            for (var iz = 0; iz < spanZ; iz++)
            {
                var z = loZ + iz;
                var index = IndexOf(x, y, z);
                var solid = hits[(ix * spanY + iy) * spanZ + iz].entityId.IsValid();

                _solid[index] = solid ? (byte)1 : (byte)0;

                ref var node = ref _nodes[index];
                node.GroundHeight = ground;
                node.WorldPosition = altitudeMode switch
                {
                    AltitudeMode.LegacyFlat => new Vector3(
                        CellCenter(x, y, z).x, ground, CellCenter(x, y, z).z),
                    AltitudeMode.Absolute => CellCenter(x, y, z),
                    _ => new Vector3(CellCenter(x, y, z).x,
                                     ground + baseHeightOffset + z * _nodeDiameter,
                                     CellCenter(x, y, z).z),
                };
                node.BasePenalty = terrainPenalty + AltitudePenalty(z);
            }
        }

        int AltitudePenalty(int z)
        {
            if (altitudeLayerPenalty == null || altitudeLayerPenalty.Length == 0) return 0;
            return altitudeLayerPenalty[Mathf.Min(z, altitudeLayerPenalty.Length - 1)];
        }

        /// <summary>
        /// Recomputes Walkable + MovementPenalty from _solid over [lo-1, hi+1]. Fully
        /// idempotent: running it twice yields the same result, so a moving obstacle that
        /// leaves a cell restores it to Passable instead of leaving a permanent scar.
        /// </summary>
        async Awaitable DilateAsync(Vector3Int lo, Vector3Int hi)
        {
            var a = Vector3Int.Max(lo - Vector3Int.one, Vector3Int.zero);
            var b = Vector3Int.Min(hi + Vector3Int.one, new Vector3Int(_sizeX - 1, _sizeY - 1, _sizeZ - 1));

            var changed = new List<int>();
            var dilate = dilateDanger;
            var surcharge = dangerPenalty;

            await Awaitable.BackgroundThreadAsync();

            ExecuteDilateLoop(a, b, changed, dilate, surcharge);

            await Awaitable.MainThreadAsync();

            if (NodeChanged != null)
                foreach (var index in changed) NodeChanged(index);
        }

        void ExecuteDilateLoop(Vector3Int a, Vector3Int b, List<int> changed, bool dilate, int surcharge)
        {
            for (var x = a.x; x <= b.x; x++)
                for (var y = a.y; y <= b.y; y++)
                    for (var z = a.z; z <= b.z; z++)
                    {
                        var index = IndexOf(x, y, z);
                        ref var node = ref _nodes[index];
                        var previous = node.Walkable;

                        Walkable state;
                        if (_solid[index] != 0) state = Walkable.Impassable;
                        else if (dilate && HasSolidNeighbour(x, y, z)) state = Walkable.Dangerous;
                        else state = Walkable.Passable;

                        node.Walkable = state;
                        node.MovementPenalty = node.BasePenalty + (state == Walkable.Dangerous ? surcharge : 0);
                        if (state != previous) changed.Add(index);
                    }
        }

        bool HasSolidNeighbour(int x, int y, int z)
        {
            for (var dx = -1; dx <= 1; dx++)
                for (var dy = -1; dy <= 1; dy++)
                    for (var dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        var nx = x + dx;
                        var ny = y + dy;
                        var nz = z + dz;
                        if (!InBounds(nx, ny, nz)) continue;
                        if (_solid[IndexOf(nx, ny, nz)] != 0) return true;
                    }

            return false;
        }

        // ---- reservations -----------------------------------------------------
        //
        // An owner id is any valid EntityId; agents pass GetEntityId(). Reservations
        // never touch Walkable, so a rebuild cannot clobber them and the owning agent
        // can plan straight through its own footprint.

        /// <summary>Owner holding this cell, or EntityId.None when free.</summary>
        public EntityId ReservedBy(int index) => _reservedBy[index];

        /// <summary>Convenience for visualizers: true when any owner holds the cell.</summary>
        public bool IsReserved(int index) => _reservedBy[index].IsValid();

        /// <summary>True if the cell blocks <paramref name="owner"/>. Own reservations don't block.</summary>
        public bool IsBlockedFor(int index, EntityId owner)
        {
            var holder = _reservedBy[index];
            return holder.IsValid() && holder != owner;
        }

        /// <summary>Traversable for this owner: not solid, and not reserved by someone else.</summary>
        public bool IsTraversableFor(int index, EntityId owner) =>
            _nodes[index].Walkable != Walkable.Impassable && !IsBlockedFor(index, owner);

        /// <summary>
        /// Replaces <paramref name="owner"/>'s entire reservation set with the given cells.
        /// Deferred if a search is reading the grid, so reservations can never race a query.
        /// </summary>
        public void Reserve(EntityId owner, IReadOnlyList<int> indices)
        {
            if (!owner.IsValid() || !IsBuilt) return;

            if (_readers > 0)
            {
                _deferredReservations.Add((-1, owner)); // sentinel: clear owner first
                if (indices != null)
                    foreach (var i in indices) _deferredReservations.Add((i, owner));
                return;
            }

            ClearOwner(owner);
            if (indices == null || indices.Count == 0) return;

            if (!_reservations.TryGetValue(owner, out var held))
                _reservations[owner] = held = new List<int>(MaxNeighbours + 1);

            foreach (var index in indices)
            {
                if ((uint)index >= (uint)_nodes.Length) continue;
                var holder = _reservedBy[index];
                if (holder.IsValid() && holder != owner) continue;
                _reservedBy[index] = owner;
                held.Add(index);
                NodeChanged?.Invoke(index);
            }
        }

        /// <summary>Releases every cell held by an owner.</summary>
        public void ClearOwner(EntityId owner)
        {
            if (!IsBuilt || !_reservations.TryGetValue(owner, out var held)) return;
            foreach (var index in held)
            {
                if (_reservedBy[index] != owner) continue;
                _reservedBy[index] = EntityId.None;
                NodeChanged?.Invoke(index);
            }

            held.Clear();
        }

        // ---- read lock --------------------------------------------------------

        public void BeginRead() => _readers++;

        public void EndRead()
        {
            _readers = Mathf.Max(0, _readers - 1);
            if (_readers != 0 || _deferredReservations.Count == 0) return;

            // replay: sentinel index -1 means "clear this owner", then re-apply
            var pending = new List<(int, EntityId)>(_deferredReservations);
            _deferredReservations.Clear();
            var buffer = new List<int>();
            var currentOwner = EntityId.None;

            foreach (var (index, owner) in pending)
            {
                if (index == -1)
                {
                    if (currentOwner.IsValid()) Reserve(currentOwner, buffer);
                    buffer.Clear();
                    currentOwner = owner;
                    continue;
                }

                buffer.Add(index);
            }

            if (currentOwner.IsValid()) Reserve(currentOwner, buffer);
        }

        // ---- neighbours -------------------------------------------------------

        /// <summary>
        /// 8 planar moves + straight up + straight down, written into <paramref name="output"/>.
        /// Returns the count. Identical connectivity to the original GetNeighbours.
        /// </summary>
        public int GetNeighbours(int index, Span<int> output)
        {
            var c = CoordsOf(index);
            var n = 0;

            for (var dx = -1; dx <= 1; dx++)
                for (var dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0) continue;
                    var nx = c.x + dx;
                    var ny = c.y + dy;
                    if (InBounds(nx, ny, c.z)) output[n++] = IndexOf(nx, ny, c.z);
                }

            if (InBounds(c.x, c.y, c.z - 1)) output[n++] = IndexOf(c.x, c.y, c.z - 1);
            if (InBounds(c.x, c.y, c.z + 1)) output[n++] = IndexOf(c.x, c.y, c.z + 1);
            return n;
        }

        /// <summary>Full 26-neighbourhood. Used for footprint reservation.</summary>
        public int GetNeighbours26(int index, Span<int> output)
        {
            var c = CoordsOf(index);
            var n = 0;
            for (var dx = -1; dx <= 1; dx++)
                for (var dy = -1; dy <= 1; dy++)
                    for (var dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        var nx = c.x + dx;
                        var ny = c.y + dy;
                        var nz = c.z + dz;
                        if (InBounds(nx, ny, nz)) output[n++] = IndexOf(nx, ny, nz);
                    }

            return n;
        }

        void OnDrawGizmosSelected()
        {
            Gizmos.color = new Color(0.3f, 0.7f, 1f, 0.4f);
            var size = new Vector3(gridWorldSize.x, gridWorldSize.z, gridWorldSize.y);
            var centre = Application.isPlaying && IsBuilt
                             ? WorldBounds.center
                             : transform.position + Vector3.up * (gridWorldSize.z * 0.5f + baseHeightOffset);
            Gizmos.DrawWireCube(centre, size);
        }
    }
}
