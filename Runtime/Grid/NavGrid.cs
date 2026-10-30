using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 均匀网格（支持多层）：区域号、代价、clearance、连通分量，以及人群密度/静止占位。
    /// </summary>
    /// <remarks>
    /// 坐标约定：世界坐标 [-WorldHalf, +WorldHalf) 映射到 [0, Side)。
    /// 层约定：层 0 为地面，其余层默认全不可通行，需由业务显式写入可走区域（如墙顶环带）。
    /// 层之间只能通过门户（<see cref="NavPortalGraph"/>）连通，A* 不允许跨层直接相邻。
    /// 只负责数据与几何查询，寻路见 <see cref="NavPathfinder"/>，代理见 <see cref="GridAgent"/>。
    /// </remarks>
    public sealed class NavGrid
    {
        /// <summary>不可通行标记。</summary>
        public const byte Impassable = byte.MaxValue;

        private readonly NavGridSettings _settings;
        private int[] _regionQueue;

        // 设置里的派生量都用 Mathf 现算（含除法），这里缓存一份：寻路内循环每帧要读几万次
        private readonly int _side;
        private readonly int _layerCells;
        private readonly int _cellCount;
        private readonly int _obstacleSide;
        private readonly float _crowdCellSize;

        /// <summary>格坐标 → 障碍快照格坐标的预计算表（避免寻路内循环里做除法）。</summary>
        private readonly int[] _crowdCellMap;

        /// <summary>区域号；<see cref="Impassable"/> 表示不可通行。索引 = 层偏移 + 层内格索引。</summary>
        public readonly byte[] Area;

        /// <summary>基础代价倍率 ×10（10 = 1.0）。</summary>
        public readonly byte[] Cost;

        /// <summary>到最近障碍的距离，单位 0.25 世界单位。</summary>
        public readonly byte[] Clearance;

        /// <summary>连通分量编号，-1 表示不可通行。跨层唯一，可直接比较。</summary>
        public readonly int[] Region;

        /// <summary>人群密度（障碍快照格，每个单位 +2）。仅用于调试热力与统计。</summary>
        public readonly byte[] Density;

        /// <summary>
        /// 静止单位的占位（障碍快照格，每个单位 +2）。
        /// 寻路把这些格当作硬障碍：站着的单位必须绕行；正在移动的单位不写入这里。
        /// </summary>
        public readonly byte[] StaticBlock;

        public NavGridSettings Settings => _settings;

        public int Side => _side;

        /// <summary>单层的格数。</summary>
        public int LayerCellCount => _layerCells;

        public int LayerCount => _settings.Layers;

        /// <summary>所有层加起来的格数。</summary>
        public int CellCount => _cellCount;

        public int RegionCount { get; private set; }

        /// <summary>网格版本，构建/更新后自增，用于缓存失效。</summary>
        public int Version { get; private set; }

        /// <summary>
        /// 当前是否登记了"站着的单位"。false 时寻路可以完全跳过静止占位判定，
        /// 省掉每个节点十几个障碍快照查询（长搜索里这是主要开销之一）。
        /// </summary>
        public bool HasStaticBlocks { get; private set; }

        public NavGrid(NavGridSettings settings)
        {
            _settings = settings ?? new NavGridSettings();
            _side = _settings.Side;
            _layerCells = _settings.CellCount;
            _cellCount = _settings.TotalCells;
            _obstacleSide = _settings.ObstacleSide;
            _crowdCellSize = Mathf.Max(_settings.CellSize, _settings.ObstacleCellSize);

            Area = new byte[_cellCount];
            Cost = new byte[_cellCount];
            Clearance = new byte[_cellCount];
            Region = new int[_cellCount];
            Density = new byte[_obstacleSide * _obstacleSide];
            StaticBlock = new byte[_obstacleSide * _obstacleSide];

            int perCell = Mathf.Max(1, _settings.CellsPerObstacleCell);
            _crowdCellMap = new int[_side];
            for (int i = 0; i < _side; i++)
            {
                _crowdCellMap[i] = Mathf.Clamp(i / perCell, 0, _obstacleSide - 1);
            }

            Clear();
        }

        #region 索引

        /// <summary>层内格索引 → 全局索引。</summary>
        public int GlobalIndex(int layer, int cellIndex)
        {
            return layer * LayerCellCount + cellIndex;
        }

        /// <summary>全局索引 → 层号。</summary>
        public int LayerOf(int globalIndex)
        {
            return globalIndex / LayerCellCount;
        }

        /// <summary>全局索引 → 层内格索引。</summary>
        public int CellOf(int globalIndex)
        {
            return globalIndex % LayerCellCount;
        }

        #endregion

        #region 构建

        public void Clear()
        {
            for (int i = 0; i < CellCount; i++)
            {
                // 地面层默认可走，其余层默认不可走（墙顶等需显式写入）
                Area[i] = i < LayerCellCount ? (byte)0 : Impassable;
                Cost[i] = 10;
                Clearance[i] = 0;
                Region[i] = -1;
            }

            RegionCount = 0;
        }

        public void BumpVersion()
        {
            Version++;
        }

        /// <summary>把世界坐标矩形（含左下、不含右上）覆盖的格标为不可通行。</summary>
        public void BlockWorldRect(float minX, float minY, float maxX, float maxY)
        {
            BlockWorldRect(0, minX, minY, maxX, maxY);
        }

        public void BlockWorldRect(int layer, float minX, float minY, float maxX, float maxY)
        {
            if (maxX <= minX || maxY <= minY)
            {
                return;
            }

            int offset = layer * LayerCellCount;
            int x0 = WorldToCellX(minX);
            int x1 = WorldToCellX(maxX - 0.0001f);
            int y0 = WorldToCellY(minY);
            int y1 = WorldToCellY(maxY - 0.0001f);

            for (int x = x0; x <= x1; x++)
            {
                int row = offset + x * Side;
                for (int y = y0; y <= y1; y++)
                {
                    Area[row + y] = Impassable;
                }
            }
        }

        /// <summary>设置单格区域（增量更新用；标为不可通行时同时清掉连通分量）。</summary>
        public void SetArea(int cellX, int cellY, byte area)
        {
            SetArea(0, cellX, cellY, area);
        }

        public void SetArea(int layer, int cellX, int cellY, byte area)
        {
            if (cellX < 0 || cellX >= Side || cellY < 0 || cellY >= Side)
            {
                return;
            }

            int index = layer * LayerCellCount + cellX * Side + cellY;
            Area[index] = area;

            if (area == Impassable)
            {
                Region[index] = -1;
            }
        }

        /// <summary>两遍 chamfer 距离变换，得到所有层的 clearance（单位 0.25 世界单位）。</summary>
        public void BuildClearance()
        {
            for (int layer = 0; layer < LayerCount; layer++)
            {
                BuildClearance(layer);
            }
        }

        public void BuildClearance(int layer)
        {
            RebuildClearanceRange(layer, 0, 0, Side - 1, Side - 1);
        }

        /// <summary>
        /// 只在给定格矩形内重算 clearance（增量更新用）。
        /// 调用方要把矩形按最大 clearance 半径外扩，矩形外的值是有效种子。
        /// </summary>
        public void RebuildClearanceLocal(int layer, int minX, int minY, int maxX, int maxY)
        {
            RebuildClearanceRange(layer, minX, minY, maxX, maxY);
        }

        private void RebuildClearanceRange(int layer, int minX, int minY, int maxX, int maxY)
        {
            // 正交/对角步长按格距换算（单位仍是 0.25 世界单位）：
            // 0.5 格 → 2 / 3，1 格 → 4 / 6。硬编码会让 CellSize 不等于 0.5 时 clearance 整体失真。
            int orth = Mathf.Max(1, Mathf.RoundToInt(_settings.CellSize * 4f));
            int diag = Mathf.Max(1, Mathf.RoundToInt(_settings.CellSize * 4f * 1.41421356f));
            int maxValue = _settings.OpenClearance;
            int offset = layer * LayerCellCount;

            bool full = minX <= 0 && minY <= 0 && maxX >= Side - 1 && maxY >= Side - 1;

            minX = Mathf.Clamp(minX, 0, Side - 1);
            maxX = Mathf.Clamp(maxX, 0, Side - 1);
            minY = Mathf.Clamp(minY, 0, Side - 1);
            maxY = Mathf.Clamp(maxY, 0, Side - 1);

            if (full)
            {
                for (int i = 0; i < LayerCellCount; i++)
                {
                    Clearance[offset + i] = Area[offset + i] == Impassable ? (byte)0 : (byte)maxValue;
                }
            }
            else
            {
                for (int x = minX; x <= maxX; x++)
                {
                    int row = offset + x * Side;
                    for (int y = minY; y <= maxY; y++)
                    {
                        int index = row + y;
                        Clearance[index] = Area[index] == Impassable ? (byte)0 : (byte)maxValue;
                    }
                }
            }

            // 正向
            for (int x = minX; x <= maxX; x++)
            {
                int row = offset + x * Side;
                for (int y = minY; y <= maxY; y++)
                {
                    int index = row + y;
                    if (Clearance[index] == 0)
                    {
                        continue;
                    }

                    int best = Clearance[index];
                    if (x > 0)
                    {
                        int left = index - Side;
                        best = Mathf.Min(best, Clearance[left] + orth);
                        if (y > 0)
                        {
                            best = Mathf.Min(best, Clearance[left - 1] + diag);
                        }

                        if (y < Side - 1)
                        {
                            best = Mathf.Min(best, Clearance[left + 1] + diag);
                        }
                    }

                    if (y > 0)
                    {
                        best = Mathf.Min(best, Clearance[index - 1] + orth);
                    }

                    Clearance[index] = (byte)Mathf.Min(best, maxValue);
                }
            }

            // 反向
            for (int x = maxX; x >= minX; x--)
            {
                int row = offset + x * Side;
                for (int y = maxY; y >= minY; y--)
                {
                    int index = row + y;
                    if (Clearance[index] == 0)
                    {
                        continue;
                    }

                    int best = Clearance[index];
                    if (x < Side - 1)
                    {
                        int right = index + Side;
                        best = Mathf.Min(best, Clearance[right] + orth);
                        if (y > 0)
                        {
                            best = Mathf.Min(best, Clearance[right - 1] + diag);
                        }

                        if (y < Side - 1)
                        {
                            best = Mathf.Min(best, Clearance[right + 1] + diag);
                        }
                    }

                    if (y < Side - 1)
                    {
                        best = Mathf.Min(best, Clearance[index + 1] + orth);
                    }

                    Clearance[index] = (byte)Mathf.Min(best, maxValue);
                }
            }
        }

        /// <summary>按 clearance 阈值做一次全图连通分量标记（跨层编号唯一）。</summary>
        public void BuildRegions(byte clearanceNeed)
        {
            int[] queue = _regionQueue ??= new int[CellCount];

            for (int i = 0; i < CellCount; i++)
            {
                Region[i] = -1;
            }

            RegionCount = 0;

            for (int layer = 0; layer < LayerCount; layer++)
            {
                int offset = layer * LayerCellCount;
                for (int cell = 0; cell < LayerCellCount; cell++)
                {
                    int start = offset + cell;
                    if (Region[start] >= 0 || Area[start] == Impassable || Clearance[start] < clearanceNeed)
                    {
                        continue;
                    }

                    FloodFillRegion(start, clearanceNeed, RegionCount++, queue);
                }
            }
        }

        /// <summary>
        /// 局部修补连通分量（增量更新用）：矩形内没有区域号的可行走格，
        /// 从邻近区域继承编号；四周都没有就新开一个区域。
        /// </summary>
        public void RepairRegions(int layer, int minX, int minY, int maxX, int maxY)
        {
            int[] queue = _regionQueue ??= new int[CellCount];
            byte clearanceNeed = _settings.RegionClearanceNeed;
            int offset = layer * LayerCellCount;

            minX = Mathf.Clamp(minX, 0, Side - 1);
            maxX = Mathf.Clamp(maxX, 0, Side - 1);
            minY = Mathf.Clamp(minY, 0, Side - 1);
            maxY = Mathf.Clamp(maxY, 0, Side - 1);

            for (int x = minX; x <= maxX; x++)
            {
                int row = offset + x * Side;
                for (int y = minY; y <= maxY; y++)
                {
                    int index = row + y;
                    if (Area[index] == Impassable || Region[index] >= 0 || Clearance[index] < clearanceNeed)
                    {
                        continue;
                    }

                    int region = FindNeighborRegion(index, x, y, minX, minY, maxX, maxY);
                    if (region < 0)
                    {
                        region = RegionCount++;
                    }

                    FloodFillRegion(index, clearanceNeed, region, queue);
                }
            }
        }

        private void FloodFillRegion(int start, byte clearanceNeed, int region, int[] queue)
        {
            int layer = LayerOf(start);
            int offset = layer * LayerCellCount;

            int head = 0;
            int tail = 0;
            queue[tail++] = start;
            Region[start] = region;

            while (head < tail)
            {
                int index = queue[head++];
                int cell = index - offset;
                int cx = cell / Side;
                int cy = cell % Side;

                for (int dx = -1; dx <= 1; dx++)
                {
                    int nx = cx + dx;
                    if (nx < 0 || nx >= Side)
                    {
                        continue;
                    }

                    for (int dy = -1; dy <= 1; dy++)
                    {
                        if (dx == 0 && dy == 0)
                        {
                            continue;
                        }

                        int ny = cy + dy;
                        if (ny < 0 || ny >= Side)
                        {
                            continue;
                        }

                        int next = offset + nx * Side + ny;
                        if (Region[next] >= 0 || Area[next] == Impassable || Clearance[next] < clearanceNeed)
                        {
                            continue;
                        }

                        Region[next] = region;
                        queue[tail++] = next;
                    }
                }
            }
        }

        private int FindNeighborRegion(int index, int x, int y, int minX, int minY, int maxX, int maxY)
        {
            int offset = LayerOf(index) * LayerCellCount;

            for (int dx = -1; dx <= 1; dx++)
            {
                int nx = x + dx;
                if (nx < minX || nx > maxX)
                {
                    continue;
                }

                for (int dy = -1; dy <= 1; dy++)
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    int ny = y + dy;
                    if (ny < minY || ny > maxY)
                    {
                        continue;
                    }

                    int neighbor = offset + nx * Side + ny;
                    if (Area[neighbor] != Impassable && Region[neighbor] >= 0)
                    {
                        return Region[neighbor];
                    }
                }
            }

            return -1;
        }

        #endregion

        #region 人群密度

        public void ClearCrowd()
        {
            System.Array.Clear(Density, 0, Density.Length);
            System.Array.Clear(StaticBlock, 0, StaticBlock.Length);
            HasStaticBlocks = false;
        }

        /// <summary>把一个单位登记到它所在的障碍快照格（每人 +2）。</summary>
        public void AddDensity(Vector2 world)
        {
            int index = CrowdIndex(world);
            if (Density[index] < byte.MaxValue - 2)
            {
                Density[index] += 2;
            }
        }

        /// <summary>
        /// 登记一个"站着不动"的单位，寻路会把它当墙。
        /// 向四邻膨胀一格：单位半径 0.5 时两者之间需要 1.0 净空，
        /// 不膨胀会在网格里留下"假缝隙"，路径就从那里钻过去。
        /// </summary>
        public void AddStaticBlock(Vector2 world)
        {
            int x = WorldToCrowdCell(world.x);
            int y = WorldToCrowdCell(world.y);

            MarkStatic(x, y);
            MarkStatic(x - 1, y);
            MarkStatic(x + 1, y);
            MarkStatic(x, y - 1);
            MarkStatic(x, y + 1);
        }

        private void MarkStatic(int x, int y)
        {
            int side = _obstacleSide;
            if (x < 0 || x >= side || y < 0 || y >= side)
            {
                return;
            }

            int index = x * side + y;
            if (StaticBlock[index] < byte.MaxValue - 2)
            {
                StaticBlock[index] += 2;
                HasStaticBlocks = true;
            }
        }

        private int CrowdIndex(Vector2 world)
        {
            int x = WorldToCrowdCell(world.x);
            int y = WorldToCrowdCell(world.y);
            return x * _obstacleSide + y;
        }

        private int WorldToCrowdCell(float world)
        {
            float size = _crowdCellSize;
            return Mathf.Clamp(Mathf.FloorToInt((world + _settings.WorldHalf) / size), 0, _obstacleSide - 1);
        }

        /// <summary>该格是否被"站着的单位"占据（寻路视作硬障碍）。人群数据不分层。</summary>
        public bool IsStaticBlocked(int globalIndex)
        {
            int cell = CellOf(globalIndex);
            return IsStaticBlockedAt(cell / _side, cell % _side);
        }

        /// <summary>
        /// 按格坐标查静止占位。寻路内循环每个节点要查多个邻格，
        /// 这里用预计算表换掉除法（<see cref="IsStaticBlocked"/> 每次要算两次除法 + 两次 Clamp）。
        /// </summary>
        public bool IsStaticBlockedAt(int cellX, int cellY)
        {
            return StaticBlock[_crowdCellMap[cellX] * _obstacleSide + _crowdCellMap[cellY]] >= 2;
        }

        #endregion

        #region 坐标

        public int WorldToCellX(float world)
        {
            return Mathf.Clamp(
                Mathf.FloorToInt((world + _settings.WorldHalf) * _settings.CellPerUnit), 0, Side - 1);
        }

        public int WorldToCellY(float world)
        {
            return WorldToCellX(world);
        }

        /// <summary>层内格索引。</summary>
        public int ToIndex(int cellX, int cellY)
        {
            return cellX * Side + cellY;
        }

        public Vector2 CellCenter(int cellX, int cellY)
        {
            float half = _settings.WorldHalf;
            float size = _settings.CellSize;
            return new Vector2((cellX + 0.5f) * size - half, (cellY + 0.5f) * size - half);
        }

        /// <summary>层内格索引对应的世界坐标格心。</summary>
        public Vector2 CellCenter(int cellIndex)
        {
            return CellCenter(cellIndex / Side, cellIndex % Side);
        }

        /// <summary>世界坐标 → 层内格索引。</summary>
        public int CellIndex(Vector2 world)
        {
            return ToIndex(WorldToCellX(world.x), WorldToCellY(world.y));
        }

        /// <summary>半径折算成 clearance 阈值（单位 0.25 世界单位）。</summary>
        public static int ClearanceNeed(float radius)
        {
            return Mathf.Max(1, Mathf.CeilToInt(radius * 4f));
        }

        #endregion

        #region 查询

        /// <summary>按全局索引判断是否可通行。</summary>
        public bool IsPassable(int globalIndex, int areaMask, float radius)
        {
            return IsPassableWithClearance(globalIndex, areaMask, ClearanceNeed(radius));
        }

        /// <summary>
        /// 按已算好的 clearance 阈值判断是否可通行。
        /// 寻路内循环里每个节点要查十几个邻格，避免重复计算 <see cref="ClearanceNeed"/>。
        /// </summary>
        public bool IsPassableWithClearance(int globalIndex, int areaMask, int clearanceNeed)
        {
            byte area = Area[globalIndex];
            if (area == Impassable || (areaMask & (1 << area)) == 0)
            {
                return false;
            }

            return Clearance[globalIndex] >= clearanceNeed;
        }

        public bool IsPassable(Vector2 world, int areaMask, float radius)
        {
            return IsPassable(0, world, areaMask, radius);
        }

        public bool IsPassable(int layer, Vector2 world, int areaMask, float radius)
        {
            return IsPassable(GlobalIndex(layer, CellIndex(world)), areaMask, radius);
        }

        /// <summary>获取世界坐标所在格的区域号；不可通行返回 -1。</summary>
        public int GetAreaIndex(Vector2 world)
        {
            return GetAreaIndex(0, world);
        }

        public int GetAreaIndex(int layer, Vector2 world)
        {
            byte area = Area[GlobalIndex(layer, CellIndex(world))];
            return area == Impassable ? -1 : area;
        }

        /// <summary>以 world 为中心，向外搜索最近的可行走格中心。</summary>
        public bool SamplePosition(Vector2 world, int areaMask, float radius, float maxRadius, out Vector2 result)
        {
            return SamplePosition(0, world, areaMask, radius, maxRadius, out result);
        }

        public bool SamplePosition(int layer, Vector2 world, int areaMask, float radius, float maxRadius,
            out Vector2 result)
        {
            result = world;

            int cx = WorldToCellX(world.x);
            int cy = WorldToCellY(world.y);
            int maxCells = Mathf.Max(1, Mathf.CeilToInt(maxRadius * _settings.CellPerUnit));
            int offset = layer * LayerCellCount;

            int bestCell = -1;
            float bestSqr = float.MaxValue;

            for (int x = Mathf.Max(0, cx - maxCells); x <= Mathf.Min(Side - 1, cx + maxCells); x++)
            {
                for (int y = Mathf.Max(0, cy - maxCells); y <= Mathf.Min(Side - 1, cy + maxCells); y++)
                {
                    int cell = x * Side + y;
                    if (!IsPassable(offset + cell, areaMask, radius))
                    {
                        continue;
                    }

                    Vector2 center = CellCenter(x, y);
                    float sqr = (center - world).sqrMagnitude;
                    if (sqr < bestSqr)
                    {
                        bestSqr = sqr;
                        bestCell = cell;
                    }
                }
            }

            if (bestCell < 0)
            {
                return false;
            }

            result = CellCenter(bestCell);
            return true;
        }

        /// <summary>廉价可达性：同层同连通分量即视为可达。</summary>
        public bool IsReachable(Vector2 from, Vector2 to, float radius)
        {
            return IsReachable(0, from, to, radius);
        }

        public bool IsReachable(int layer, Vector2 from, Vector2 to, float radius)
        {
            int need = ClearanceNeed(radius);
            int offset = layer * LayerCellCount;
            int startIndex = offset + CellIndex(from);
            int goalIndex = offset + CellIndex(to);

            if (Clearance[startIndex] < need || Clearance[goalIndex] < need)
            {
                return false;
            }

            int region = Region[startIndex];
            return region >= 0 && region == Region[goalIndex];
        }

        #endregion
    }
}
