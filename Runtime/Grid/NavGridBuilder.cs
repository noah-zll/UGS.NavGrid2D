using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 把业务登记的障碍写进 <see cref="NavGrid"/>，并支持差异增量重建（按层独立判定脏矩形）。
    /// </summary>
    /// <remarks>
    /// 每次 <see cref="Build"/> 都会重新光栅化全部障碍源，与上一次快照比较：
    /// 没有变化直接返回；变化小则只重算脏矩形内的 area / clearance / region；变化大或首次构建才全量重建。
    /// </remarks>
    public sealed class NavGridBuilder
    {
        private const int FullRebuildThreshold = 20000;

        private readonly NavGridSettings _settings;
        private readonly byte[] _snapshot;
        private readonly byte[] _current;
        private readonly NavGridObstacleMap _map;

        private readonly int[] _dirtyMinX;
        private readonly int[] _dirtyMinY;
        private readonly int[] _dirtyMaxX;
        private readonly int[] _dirtyMaxY;

        private bool _hasSnapshot;

        public NavGridBuilder(NavGridSettings settings)
        {
            _settings = settings ?? new NavGridSettings();

            int layerCount = _settings.Layers;
            int count = _settings.ObstacleSide * _settings.ObstacleSide * layerCount;
            _snapshot = new byte[count];
            _current = new byte[count];
            _map = new NavGridObstacleMap(_settings, _current);

            _dirtyMinX = new int[layerCount];
            _dirtyMinY = new int[layerCount];
            _dirtyMaxX = new int[layerCount];
            _dirtyMaxY = new int[layerCount];
        }

        /// <summary>换图/清空网格时调用，强制下次走全量重建。</summary>
        public void Reset()
        {
            _hasSnapshot = false;
            System.Array.Clear(_snapshot, 0, _snapshot.Length);
        }

        /// <summary>
        /// 重建网格，返回本次改动的障碍格数量（0 表示没有任何变化，网格版本不变）。
        /// </summary>
        public int Build(NavGrid grid, IReadOnlyList<INavGridObstacleSource> sources)
        {
            if (grid == null)
            {
                return 0;
            }

            _map.Clear();

            if (sources != null)
            {
                for (int i = 0; i < sources.Count; i++)
                {
                    sources[i]?.Rasterize(_map);
                }
            }

            if (!_hasSnapshot)
            {
                FullRebuild(grid);
                System.Array.Copy(_current, _snapshot, _current.Length);
                _hasSnapshot = true;
                return _current.Length;
            }

            int side = _settings.ObstacleSide;
            int layerCount = _settings.Layers;
            int layerCells = side * side;
            int changed = 0;
            bool opened = false;
            bool closed = false;

            for (int layer = 0; layer < layerCount; layer++)
            {
                _dirtyMinX[layer] = side;
                _dirtyMinY[layer] = side;
                _dirtyMaxX[layer] = -1;
                _dirtyMaxY[layer] = -1;
            }

            for (int i = 0; i < _current.Length; i++)
            {
                if (_current[i] == _snapshot[i])
                {
                    continue;
                }

                changed++;

                int layer = i / layerCells;
                int cell = i - layer * layerCells;
                int x = cell / side;
                int y = cell % side;

                // 阻挡 → 可走：原本被墙隔开的两片区域会真正连通，区域号必须重新合并
                if (!opened && !IsPassableValue(layer, _snapshot[i]) && IsPassableValue(layer, _current[i]))
                {
                    opened = true;
                }
                // 可走 → 阻挡：一片区域会被切成两片，旧的区域号会把两侧仍判成"同一分量"
                else if (!closed && IsPassableValue(layer, _snapshot[i]) && !IsPassableValue(layer, _current[i]))
                {
                    closed = true;
                }

                if (x < _dirtyMinX[layer]) _dirtyMinX[layer] = x;
                if (x > _dirtyMaxX[layer]) _dirtyMaxX[layer] = x;
                if (y < _dirtyMinY[layer]) _dirtyMinY[layer] = y;
                if (y > _dirtyMaxY[layer]) _dirtyMaxY[layer] = y;
            }

            if (changed == 0)
            {
                return 0;
            }

            if (changed > FullRebuildThreshold)
            {
                FullRebuild(grid);
            }
            else
            {
                for (int layer = 0; layer < layerCount; layer++)
                {
                    if (_dirtyMaxX[layer] < 0)
                    {
                        continue;
                    }

                    ApplyIncremental(grid, layer, _dirtyMinX[layer], _dirtyMinY[layer], _dirtyMaxX[layer],
                        _dirtyMaxY[layer]);
                }

                // RepairRegions 只会给"新可走"的格补区域号，既不会合并两侧区域，也不会拆分：
                // 打通（开门/开洞）后要合并，阻挡（阵法/建筑落地）后要拆分，两种都会让旧区域号失效，
                // 不整体重算就会出现"区域号相同但根本走不过去"（IsReachable 判可达、A* 却到不了）。
                if (opened || closed)
                {
                    grid.BuildRegions(_settings.RegionClearanceNeed);
                }
            }

            System.Array.Copy(_current, _snapshot, _current.Length);
            return changed;
        }

        private void FullRebuild(NavGrid grid)
        {
            grid.Clear();

            int side = _settings.ObstacleSide;
            int perCell = _settings.CellsPerObstacleCell;
            int layerCells = side * side;

            for (int layer = 0; layer < _settings.Layers; layer++)
            {
                int offset = layer * layerCells;

                for (int x = 0; x < side; x++)
                {
                    int row = offset + x * side;
                    for (int y = 0; y < side; y++)
                    {
                        if (_current[row + y] == 0)
                        {
                            continue;
                        }

                        byte area = ResolveArea(layer, _current[row + y]);
                        int gx = x * perCell;
                        int gy = y * perCell;
                        for (int dx = 0; dx < perCell; dx++)
                        {
                            for (int dy = 0; dy < perCell; dy++)
                            {
                                grid.SetArea(layer, gx + dx, gy + dy, area);
                            }
                        }
                    }
                }
            }

            grid.BuildClearance();
            grid.BuildRegions(_settings.RegionClearanceNeed);
            grid.BumpVersion();
        }

        private void ApplyIncremental(NavGrid grid, int layer, int minX, int minY, int maxX, int maxY)
        {
            int side = _settings.ObstacleSide;
            int perCell = _settings.CellsPerObstacleCell;
            int offset = layer * side * side;

            for (int x = minX; x <= maxX; x++)
            {
                int row = offset + x * side;
                for (int y = minY; y <= maxY; y++)
                {
                    int index = row + y;
                    if (_current[index] == _snapshot[index])
                    {
                        continue;
                    }

                    byte area = ResolveArea(layer, _current[index]);
                    int gx = x * perCell;
                    int gy = y * perCell;

                    for (int dx = 0; dx < perCell; dx++)
                    {
                        for (int dy = 0; dy < perCell; dy++)
                        {
                            grid.SetArea(layer, gx + dx, gy + dy, area);
                        }
                    }
                }
            }

            int margin = _settings.ClearanceMargin;
            int gMinX = minX * perCell - margin;
            int gMinY = minY * perCell - margin;
            int gMaxX = (maxX + 1) * perCell + margin;
            int gMaxY = (maxY + 1) * perCell + margin;

            grid.RebuildClearanceLocal(layer, gMinX, gMinY, gMaxX, gMaxY);
            grid.RepairRegions(layer, gMinX, gMinY, gMaxX, gMaxY);
            grid.BumpVersion();
        }

        /// <summary>快照值 → 区域号：0 = 该层默认，1 = 阻挡，≥2 = 区域号 + 2。</summary>
        private static byte ResolveArea(int layer, byte value)
        {
            if (value == 0)
            {
                return layer == 0 ? (byte)0 : NavGrid.Impassable;
            }

            return value == 1 ? NavGrid.Impassable : (byte)(value - 2);
        }

        /// <summary>快照值在该层是否为可走格。</summary>
        private static bool IsPassableValue(int layer, byte value)
        {
            return ResolveArea(layer, value) != NavGrid.Impassable;
        }
    }
}
