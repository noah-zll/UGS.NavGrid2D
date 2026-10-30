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
        /// 同一层内两点之间是否全程可通行（按半个格距采样）。
        /// </summary>
        /// <remarks>
        /// 除了静态可走，还要避开被单位占据的格：否则 A* 绕开的人群会被平滑一步拉直穿过去。
        /// 路径起点/终点附近放行，避免被自己或目标那个人卡死。
        /// </remarks>
        public bool HasLineOfSight(NavGrid grid, int layer, Vector2 from, Vector2 to, int areaMask,
            float radius, Vector2 pathStart, Vector2 pathEnd, bool ignoreCrowd)
        {
            float distance = Vector2.Distance(from, to);
            int steps = Mathf.CeilToInt(distance / (grid.Settings.CellSize * 0.5f));
            if (steps <= 0)
            {
                return true;
            }

            float exemptSqr = EndpointExemptRadius * EndpointExemptRadius;
            int layerOffset = layer * grid.LayerCellCount;

            for (int i = 0; i <= steps; i++)
            {
                Vector2 point = Vector2.Lerp(from, to, (float)i / steps);
                if (!grid.IsPassable(layer, point, areaMask, radius))
                {
                    return false;
                }

                if (ignoreCrowd)
                {
                    continue;
                }

                if ((point - pathStart).sqrMagnitude <= exemptSqr ||
                    (point - pathEnd).sqrMagnitude <= exemptSqr)
                {
                    continue;
                }

                if (grid.IsStaticBlocked(layerOffset + grid.CellIndex(point)))
                {
                    return false;
                }
            }

            return true;
        }
    }
}
