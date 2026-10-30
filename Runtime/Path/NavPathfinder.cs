using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 8 向网格 A*（支持多层）。对角线不允许穿角；层之间只能通过门户穿越。
    /// 终点不可达时返回离终点最近的已展开点。
    /// </summary>
    /// <remarks>
    /// 所有 scratch（gScore/parent/visit/堆/格序列/平滑缓冲）都是实例字段：
    /// 主线程走静态 API（用 <c>Default</c> 实例），后台寻路每个线程各持一个实例，
    /// 这样多个搜索并行时不会互相踩内存。同一个实例不可重入。
    /// </remarks>
    public sealed class NavPathfinder
    {
        private const float DiagonalLength = 1.41421356f;

        /// <summary>起点/终点周围不做"单位阻挡"的半径（世界单位）。</summary>
        private const float EndpointExemptRadius = 1.5f;

        /// <summary>门户穿越的最小代价，避免跨层变成免费捷径。</summary>
        private const float MinPortalCost = 0.5f;

        private readonly PathSmootherContext _smoother = new PathSmootherContext();

        private float[] _gScore;
        private int[] _parent;
        private int[] _visit;
        private int _scratchSize;
        private int _token;

        private int[] _heapIndex = new int[4096];
        private float[] _heapScore = new float[4096];
        private int _heapCount;

        private readonly List<int> _cellPath = new List<int>(256);

        /// <summary>当前这次搜索的启发值权重（单次请求可覆盖）。</summary>
        private float _heuristicWeight = 1.02f;

        private static readonly int[] NeighborDx = { 1, -1, 0, 0, 1, 1, -1, -1 };
        private static readonly int[] NeighborDy = { 0, 0, 1, -1, 1, -1, 1, -1 };

        /// <summary>静态 API 用的主线程实例。后台线程不要共用它。</summary>
        private static readonly NavPathfinder DefaultInstance = new NavPathfinder();

        /// <summary>最近一次寻路的展开次数。</summary>
        public int Expansions { get; private set; }

        /// <summary>最近一次寻路是否真正走到目标（false = 返回的是"最近已展开点"）。</summary>
        public bool ReachedGoal { get; private set; }

        /// <summary>
        /// 最近一次寻路是否被展开上限截断（true = 还有没搜完的区域；false 且未到达目标 = 目标确实不可达）。
        /// </summary>
        public bool Truncated { get; private set; }

        /// <summary>最近一次寻路耗时（毫秒）。</summary>
        public float Milliseconds { get; private set; }

        /// <summary>最近一次寻路的展开次数（主线程）。</summary>
        public static int LastExpansions => DefaultInstance.Expansions;

        /// <summary>最近一次寻路是否真正走到目标（主线程）。</summary>
        public static bool LastReachedGoal => DefaultInstance.ReachedGoal;

        /// <summary>最近一次寻路是否被展开上限截断（主线程）。</summary>
        public static bool LastTruncated => DefaultInstance.Truncated;

        /// <summary>最近一次寻路耗时（毫秒，主线程）。</summary>
        public static float LastMilliseconds => DefaultInstance.Milliseconds;

        /// <summary>
        /// 求解从 start 到 goal 的路径（世界坐标 + 层号），主线程静态入口。
        /// </summary>
        /// <param name="portals">门户图，可为 null（此时不跨层）。</param>
        /// <param name="corners">输出：平滑后的路点（不含起点），跨层处相邻两点层号不同。</param>
        /// <param name="ignoreCrowd">true 时忽略"站着的单位"阻挡（阵型整体推进用）。</param>
        public static NavPathStatus FindPath(NavGrid grid, INavPortalQuery portals,
            int startLayer, Vector2 start, int goalLayer, Vector2 goal,
            int areaMask, float radius, List<NavPathPoint> corners, bool ignoreCrowd, int maxExpansions = 0,
            float maxMilliseconds = 0f, float heuristicWeight = 0f)
        {
            return DefaultInstance.Search(grid, portals, startLayer, start, goalLayer, goal, areaMask, radius,
                corners, ignoreCrowd, maxExpansions, maxMilliseconds, heuristicWeight, null);
        }

        /// <summary>
        /// 用本实例的 scratch 求一次路径（后台线程用）。
        /// </summary>
        /// <param name="owner">可选：任务被取消时提前退出搜索。</param>
        public NavPathStatus Search(NavGrid grid, INavPortalQuery portals,
            int startLayer, Vector2 start, int goalLayer, Vector2 goal,
            int areaMask, float radius, List<NavPathPoint> corners, bool ignoreCrowd, int maxExpansions = 0,
            float maxMilliseconds = 0f, float heuristicWeight = 0f, NavPathJob owner = null)
        {
            Expansions = 0;
            ReachedGoal = false;
            Truncated = false;

            long startTimestamp = System.Diagnostics.Stopwatch.GetTimestamp();
            NavPathStatus status = SearchInternal(grid, portals, startLayer, start, goalLayer, goal,
                areaMask, radius, corners, ignoreCrowd, maxExpansions, maxMilliseconds, heuristicWeight, owner);
            Milliseconds = (System.Diagnostics.Stopwatch.GetTimestamp() - startTimestamp) * 1000f /
                           System.Diagnostics.Stopwatch.Frequency;
            return status;
        }

        private NavPathStatus SearchInternal(NavGrid grid, INavPortalQuery portals,
            int startLayer, Vector2 start, int goalLayer, Vector2 goal,
            int areaMask, float radius, List<NavPathPoint> corners, bool ignoreCrowd, int maxExpansions,
            float maxMilliseconds, float heuristicWeight, NavPathJob owner)
        {
            corners.Clear();
            if (grid == null)
            {
                return NavPathStatus.Invalid;
            }

            EnsureScratch(grid.CellCount);

            NavGridSettings settings = grid.Settings;
            int side = grid.Side;
            int layerCells = grid.LayerCellCount;
            float cellSize = settings.CellSize;

            startLayer = Mathf.Clamp(startLayer, 0, grid.LayerCount - 1);
            goalLayer = Mathf.Clamp(goalLayer, 0, grid.LayerCount - 1);

            int startIndex = grid.GlobalIndex(startLayer, grid.CellIndex(start));
            if (!grid.IsPassable(startIndex, areaMask, radius))
            {
                if (!grid.SamplePosition(startLayer, start, areaMask, radius, radius + 1f, out Vector2 sampledStart))
                {
                    return NavPathStatus.Invalid;
                }

                startIndex = grid.GlobalIndex(startLayer, grid.CellIndex(sampledStart));
            }

            int goalIndex = grid.GlobalIndex(goalLayer, grid.CellIndex(goal));
            bool goalPassable = grid.IsPassable(goalIndex, areaMask, radius);
            Vector2 finalGoal = goal;

            if (!goalPassable)
            {
                // 目标点本身不可走：先找附近最近可走格；找不到就留给 A* 用"最近已展开点"收尾
                if (grid.SamplePosition(goalLayer, goal, areaMask, radius, settings.NearestSearchRadius,
                        out Vector2 sampledGoal))
                {
                    goalIndex = grid.GlobalIndex(goalLayer, grid.CellIndex(sampledGoal));
                    finalGoal = sampledGoal;
                    goalPassable = true;
                }
            }

            if (startIndex == goalIndex)
            {
                return NavPathStatus.Complete;
            }

            _token++;
            if (_token == int.MaxValue)
            {
                System.Array.Clear(_visit, 0, _visit.Length);
                _token = 1;
            }

            _heapCount = 0;
            _visit[startIndex] = _token;
            _parent[startIndex] = -1;
            _gScore[startIndex] = 0f;

            // 内循环热路径：阈值算一次，邻格判定走轻量重载（每个节点要查十几个邻格）
            int clearanceNeed = NavGrid.ClearanceNeed(radius);

            // 单次请求可以覆盖全局展开/耗时/启发权重（玩家指令或"预算内搜不完"的重试）
            int expansionLimit = maxExpansions > 0 ? maxExpansions : settings.MaxExpansions;
            float timeBudgetMs = maxMilliseconds > 0f ? maxMilliseconds : settings.MaxPathMilliseconds;
            _heuristicWeight = heuristicWeight > 0f ? heuristicWeight : settings.HeuristicWeight;
            long timeLimitTicks = timeBudgetMs > 0f
                ? (long)(timeBudgetMs * System.Diagnostics.Stopwatch.Frequency / 1000.0)
                : long.MaxValue;
            long searchStart = System.Diagnostics.Stopwatch.GetTimestamp();
            bool timeExpired = false;

            // 启发值直接按格坐标算（不再从全局索引反解坐标），比例在循环外合成一次
            int goalCell = goalIndex % layerCells;
            int goalX = goalCell / side;
            int goalY = goalCell % side;
            float heuristicScale = _heuristicWeight * cellSize;

            // 没有"站着的单位"时整段静止占位判定都可以跳过
            bool useCrowd = !ignoreCrowd && grid.HasStaticBlocks;

            int startCell = startIndex % layerCells;
            int startX = startCell / side;
            int startY = startCell % side;
            Push(startIndex, Heuristic(startX, startY, goalX, goalY) * heuristicScale);

            int bestIndex = startIndex;
            float bestHeuristic = Heuristic(startX, startY, goalX, goalY);
            int expansions = 0;
            bool reached = false;

            float exemptSqr = EndpointExemptRadius * EndpointExemptRadius;

            while (_heapCount > 0 && expansions < expansionLimit)
            {
                // 每 1024 次展开查一次表：长绕行靠时间封顶，避免一次请求拖十几毫秒
                if ((expansions & 1023) == 0)
                {
                    if (System.Diagnostics.Stopwatch.GetTimestamp() - searchStart > timeLimitTicks)
                    {
                        timeExpired = true;
                        break;
                    }

                    if (owner != null && owner.IsCancelled)
                    {
                        timeExpired = true;
                        break;
                    }
                }

                int current = Pop();

                // 惰性删除：同一节点可能被多次入堆
                if (_visit[current] == -_token)
                {
                    continue;
                }

                if (_visit[current] != _token)
                {
                    continue;
                }

                _visit[current] = -_token;
                expansions++;

                if (current == goalIndex && goalPassable)
                {
                    reached = true;
                    break;
                }

                int layerOffset = current / layerCells * layerCells;
                int cell = current - layerOffset;
                int cx = cell / side;
                int cy = cell % side;

                float heuristic = Heuristic(cx, cy, goalX, goalY);
                if (heuristic < bestHeuristic)
                {
                    bestHeuristic = heuristic;
                    bestIndex = current;
                }

                for (int i = 0; i < 8; i++)
                {
                    int nx = cx + NeighborDx[i];
                    int ny = cy + NeighborDy[i];
                    if (nx < 0 || nx >= side || ny < 0 || ny >= side)
                    {
                        continue;
                    }

                    int next = layerOffset + nx * side + ny;
                    if (!grid.IsPassableWithClearance(next, areaMask, clearanceNeed))
                    {
                        continue;
                    }

                    // 站着的单位当硬障碍：路径必须绕行；移动中的单位不写入阻挡。
                    // 起点/终点附近豁免，否则会被自己或目标那个人卡死。
                    if (useCrowd && grid.IsStaticBlockedAt(nx, ny) &&
                        !IsNearEndpoint(grid, next, start, goal, exemptSqr))
                    {
                        continue;
                    }

                    if (NeighborDx[i] != 0 && NeighborDy[i] != 0)
                    {
                        // 禁止穿角：两个正交格都必须可走
                        if (!grid.IsPassableWithClearance(layerOffset + cx * side + ny, areaMask, clearanceNeed) ||
                            !grid.IsPassableWithClearance(layerOffset + nx * side + cy, areaMask, clearanceNeed))
                        {
                            continue;
                        }
                    }

                    if (_visit[next] == -_token)
                    {
                        continue;
                    }

                    float step = NeighborDx[i] != 0 && NeighborDy[i] != 0 ? DiagonalLength : 1f;
                    float cost = _gScore[current] + step * cellSize * (grid.Cost[next] / 10f);

                    if (_visit[next] != _token || cost < _gScore[next])
                    {
                        _gScore[next] = cost;
                        _parent[next] = current;
                        _visit[next] = _token;
                        Push(next, cost + Heuristic(nx, ny, goalX, goalY) * heuristicScale);
                    }
                }

                // 门户：层之间唯一的连通方式
                if (portals != null && portals.Count > 0)
                {
                    IReadOnlyList<int> portalIndices = portals.Query(current);
                    for (int p = 0; p < portalIndices.Count; p++)
                    {
                        NavPortal portal = portals[portalIndices[p]];
                        if (!portals.TryTraverse(portal, current, out int next))
                        {
                            continue;
                        }

                        if (next == current || next < 0 || next >= grid.CellCount)
                        {
                            continue;
                        }

                        if (!grid.IsPassableWithClearance(next, areaMask, clearanceNeed))
                        {
                            continue;
                        }

                        if (_visit[next] == -_token)
                        {
                            continue;
                        }

                        Vector2 from = grid.CellCenter(grid.CellOf(current));
                        Vector2 to = grid.CellCenter(grid.CellOf(next));
                        float speed = Mathf.Max(0.5f, portal.TraverseSpeed);
                        float cost = _gScore[current] + Mathf.Max(Vector2.Distance(from, to) / speed, MinPortalCost);

                        if (_visit[next] != _token || cost < _gScore[next])
                        {
                            _gScore[next] = cost;
                            _parent[next] = current;
                            _visit[next] = _token;
                            int nextCell = grid.CellOf(next);
                            Push(next, cost + Heuristic(nextCell / side, nextCell % side, goalX, goalY) * heuristicScale);
                        }
                    }
                }
            }

            Expansions = expansions;
            ReachedGoal = reached;
            Truncated = !reached && (timeExpired || expansions >= expansionLimit);

            _cellPath.Clear();
            int walk = reached ? goalIndex : bestIndex;
            int guard = 0;
            while (walk >= 0 && guard++ < grid.CellCount)
            {
                _cellPath.Add(walk);
                if (walk == startIndex)
                {
                    break;
                }

                walk = _parent[walk];
            }

            _cellPath.Reverse();

            if (_cellPath.Count == 0)
            {
                return NavPathStatus.Invalid;
            }

            if (_cellPath.Count > 256)
            {
                // 超长路径不做平滑，避免 O(n²) 采样尖峰
                for (int i = 1; i < _cellPath.Count; i++)
                {
                    int index = _cellPath[i];
                    corners.Add(new NavPathPoint(grid.CellCenter(grid.CellOf(index)), grid.LayerOf(index)));
                }
            }
            else
            {
                _smoother.Smooth(grid, _cellPath, areaMask, radius, corners, ignoreCrowd);
                if (corners.Count > 0)
                {
                    corners.RemoveAt(0);
                }
            }

            if (reached)
            {
                // 终点用精确目标点，避免吸附到格心
                if (corners.Count > 0)
                {
                    corners[corners.Count - 1] = new NavPathPoint(finalGoal, goalLayer);
                }
                else
                {
                    corners.Add(new NavPathPoint(finalGoal, goalLayer));
                }

                return NavPathStatus.Complete;
            }

            if (corners.Count == 0)
            {
                return NavPathStatus.Invalid;
            }

            return NavPathStatus.Partial;
        }

        private static bool IsNearEndpoint(NavGrid grid, int globalIndex, Vector2 start, Vector2 goal, float exemptSqr)
        {
            Vector2 center = grid.CellCenter(grid.CellOf(globalIndex));
            return (center - start).sqrMagnitude <= exemptSqr || (center - goal).sqrMagnitude <= exemptSqr;
        }

        private static float Heuristic(int cx, int cy, int goalX, int goalY)
        {
            int dx = cx > goalX ? cx - goalX : goalX - cx;
            int dy = cy > goalY ? cy - goalY : goalY - cy;
            int min = dx < dy ? dx : dy;
            int max = dx < dy ? dy : dx;
            return max - min + DiagonalLength * min;
        }

        private void EnsureScratch(int cellCount)
        {
            if (_scratchSize == cellCount)
            {
                return;
            }

            _scratchSize = cellCount;
            _gScore = new float[cellCount];
            _parent = new int[cellCount];
            _visit = new int[cellCount];
            _token = 0;
        }

        #region 最小堆（惰性删除）

        private void Push(int index, float score)
        {
            if (_heapCount == _heapIndex.Length)
            {
                System.Array.Resize(ref _heapIndex, _heapIndex.Length * 2);
                System.Array.Resize(ref _heapScore, _heapScore.Length * 2);
            }

            int position = _heapCount++;
            while (position > 0)
            {
                int parentPosition = (position - 1) >> 1;
                if (_heapScore[parentPosition] <= score)
                {
                    break;
                }

                _heapIndex[position] = _heapIndex[parentPosition];
                _heapScore[position] = _heapScore[parentPosition];
                position = parentPosition;
            }

            _heapIndex[position] = index;
            _heapScore[position] = score;
        }

        private int Pop()
        {
            int result = _heapIndex[0];
            _heapCount--;

            if (_heapCount > 0)
            {
                int index = _heapIndex[_heapCount];
                float score = _heapScore[_heapCount];
                int position = 0;

                while (true)
                {
                    int child = position * 2 + 1;
                    if (child >= _heapCount)
                    {
                        break;
                    }

                    if (child + 1 < _heapCount && _heapScore[child + 1] < _heapScore[child])
                    {
                        child++;
                    }

                    if (_heapScore[child] >= score)
                    {
                        break;
                    }

                    _heapIndex[position] = _heapIndex[child];
                    _heapScore[position] = _heapScore[child];
                    position = child;
                }

                _heapIndex[position] = index;
                _heapScore[position] = score;
            }

            return result;
        }

        #endregion
    }
}
