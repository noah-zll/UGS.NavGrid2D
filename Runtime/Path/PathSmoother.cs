using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>网格路径平滑：string pulling + 视线（LOS）校验，按层分段处理。</summary>
    /// <remarks>跨层处（相邻格层号不同，即门户穿越）强制断开，不做跨层拉直。</remarks>
    public static class PathSmoother
    {
        /// <summary>主线程共用实例。后台寻路线程各自持有 <see cref="PathSmootherContext"/>，不会走这里。</summary>
        private static readonly PathSmootherContext Default = new PathSmootherContext();

        /// <summary>把格序列平滑成路点（含起点）。</summary>
        public static void Smooth(NavGrid grid, List<int> cells, int areaMask, float radius,
            List<NavPathPoint> result, bool ignoreCrowd)
        {
            Default.Smooth(grid, cells, areaMask, radius, result, ignoreCrowd);
        }

        /// <summary>同一层内两点之间是否全程可通行（按半个格距采样）。</summary>
        public static bool HasLineOfSight(NavGrid grid, int layer, Vector2 from, Vector2 to, int areaMask,
            float radius, Vector2 pathStart, Vector2 pathEnd, bool ignoreCrowd)
        {
            return Default.HasLineOfSight(grid, layer, from, to, areaMask, radius, pathStart, pathEnd, ignoreCrowd);
        }
    }

    /// <summary>
    /// 平滑用的可复用缓冲。
    /// 原来这些列表是 static，后台线程并行寻路时会互相覆盖，所以抽成实例：
    /// 一个线程一个实例，静态 API 用主线程那一份。
    /// </summary>
    internal sealed class PathSmootherContext
    {
        /// <summary>起点/终点周围不做单位阻挡的半径（世界单位）。</summary>
        private const float EndpointExemptRadius = 1.5f;

        private readonly List<Vector2> _rawPoints = new List<Vector2>(512);
        private readonly List<int> _rawLayers = new List<int>(512);

        public void Smooth(NavGrid grid, List<int> cells, int areaMask, float radius,
            List<NavPathPoint> result, bool ignoreCrowd)
        {
            result.Clear();

            int count = cells.Count;
            if (count == 0)
            {
                return;
            }

            _rawPoints.Clear();
            _rawLayers.Clear();
            for (int i = 0; i < count; i++)
            {
                int index = cells[i];
                _rawPoints.Add(grid.CellCenter(grid.CellOf(index)));
                _rawLayers.Add(grid.LayerOf(index));
            }

            if (count == 1)
            {
                result.Add(new NavPathPoint(_rawPoints[0], _rawLayers[0]));
                return;
            }

            Vector2 pathStart = _rawPoints[0];
            Vector2 pathEnd = _rawPoints[count - 1];

            int runStart = 0;
            while (runStart < count)
            {
                int layer = _rawLayers[runStart];
                int runEnd = runStart;
                while (runEnd + 1 < count && _rawLayers[runEnd + 1] == layer)
                {
                    runEnd++;
                }

                // 每段起点都要保留：跨层段的第一个点就是门户落地格
                result.Add(new NavPathPoint(_rawPoints[runStart], layer));

                int anchor = runStart;
                while (anchor < runEnd)
                {
                    int farthest = anchor + 1;
                    for (int probe = runEnd; probe > anchor + 1; probe--)
                    {
                        if (HasLineOfSight(grid, layer, _rawPoints[anchor], _rawPoints[probe], areaMask, radius,
                                pathStart, pathEnd, ignoreCrowd))
                        {
                            farthest = probe;
                            break;
                        }
                    }

                    result.Add(new NavPathPoint(_rawPoints[farthest], layer));
                    anchor = farthest;
                }

                runStart = runEnd + 1;
            }
        }

        /// <summary>
        /// 同一层内两点之间是否全程可通行。
        /// </summary>
        /// <remarks>
        /// 逐格穿过（DDA）而不是按固定步长采样：采样会漏掉"擦着建筑角"的情况
        /// ——两端采样点都落在可走格，但线段刚好切过挡块的一角，
        /// 单位走过去后当前格被判不可走，上层把它纠正回最近可走格，表现为原地抽搐。
        /// 除了静态可走，还要避开被单位占据的格：否则 A* 绕开的人群会被平滑一步拉直穿过去。
        /// 路径起点/终点附近放行，避免被自己或目标那个人卡死。
        /// </remarks>
        public bool HasLineOfSight(NavGrid grid, int layer, Vector2 from, Vector2 to, int areaMask,
            float radius, Vector2 pathStart, Vector2 pathEnd, bool ignoreCrowd)
        {
            float exemptSqr = EndpointExemptRadius * EndpointExemptRadius;
            int layerOffset = layer * grid.LayerCellCount;
            int side = grid.Side;
            int clearanceNeed = NavGrid.ClearanceNeed(radius);

            bool CellPassable(int cx, int cy)
            {
                if (!grid.IsPassableWithClearance(layerOffset + cx * side + cy, areaMask, clearanceNeed))
                {
                    return false;
                }

                if (ignoreCrowd)
                {
                    return true;
                }

                Vector2 center = grid.CellCenter(cx, cy);
                if ((center - pathStart).sqrMagnitude <= exemptSqr ||
                    (center - pathEnd).sqrMagnitude <= exemptSqr)
                {
                    return true;
                }

                return !grid.IsStaticBlockedAt(cx, cy);
            }

            float cellSize = grid.Settings.CellSize;
            float halfWorld = grid.Settings.WorldHalf;

            // 世界坐标 → 连续格坐标（格 i 覆盖 [i, i+1)）
            float fx0 = (from.x + halfWorld) / cellSize;
            float fy0 = (from.y + halfWorld) / cellSize;
            float fx1 = (to.x + halfWorld) / cellSize;
            float fy1 = (to.y + halfWorld) / cellSize;

            int x = Mathf.Clamp(Mathf.FloorToInt(fx0), 0, side - 1);
            int y = Mathf.Clamp(Mathf.FloorToInt(fy0), 0, side - 1);
            int endX = Mathf.Clamp(Mathf.FloorToInt(fx1), 0, side - 1);
            int endY = Mathf.Clamp(Mathf.FloorToInt(fy1), 0, side - 1);

            float dx = fx1 - fx0;
            float dy = fy1 - fy0;
            int stepX = dx >= 0f ? 1 : -1;
            int stepY = dy >= 0f ? 1 : -1;
            float tDeltaX = dx != 0f ? Mathf.Abs(1f / dx) : float.PositiveInfinity;
            float tDeltaY = dy != 0f ? Mathf.Abs(1f / dy) : float.PositiveInfinity;
            float tMaxX = dx != 0f
                ? (stepX > 0 ? x + 1 - fx0 : fx0 - x) / Mathf.Abs(dx)
                : float.PositiveInfinity;
            float tMaxY = dy != 0f
                ? (stepY > 0 ? y + 1 - fy0 : fy0 - y) / Mathf.Abs(dy)
                : float.PositiveInfinity;

            // 正常走过格数上限；多余几次留给浮点误差
            int guard = Mathf.Abs(endX - x) + Mathf.Abs(endY - y) + 4;

            while (true)
            {
                if (!CellPassable(x, y))
                {
                    return false;
                }

                if (x == endX && y == endY)
                {
                    return true;
                }

                if (guard-- <= 0)
                {
                    return true;
                }

                if (tMaxX < tMaxY - 1e-6f)
                {
                    x += stepX;
                    tMaxX += tDeltaX;
                }
                else if (tMaxY < tMaxX - 1e-6f)
                {
                    y += stepY;
                    tMaxY += tDeltaY;
                }
                else
                {
                    // 线段正好穿过四格交界的角点：另外两个正交格也要能站住，
                    // 否则这一步就是"贴着挡块角挤过去"，实际走不了
                    if (x + stepX < 0 || x + stepX >= side || y + stepY < 0 || y + stepY >= side ||
                        !CellPassable(x + stepX, y) || !CellPassable(x, y + stepY))
                    {
                        return false;
                    }

                    x += stepX;
                    y += stepY;
                    tMaxX += tDeltaX;
                    tMaxY += tDeltaY;
                }

                if (x < 0 || x >= side || y < 0 || y >= side)
                {
                    return false;
                }
            }
        }
    }
}
