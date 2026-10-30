using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 静态障碍来源。业务侧实现它，把「当前有哪些格子被挡住」写进快照。
    /// </summary>
    /// <remarks>
    /// 每次重建都会重新调用；框架负责和上一次快照做差异，只有变化区域会重算。
    /// 只写地面层时用不带 layer 的重载即可。
    /// </remarks>
    public interface INavGridObstacleSource
    {
        /// <summary>把当前所有静态障碍写入 <paramref name="map"/>（1 = 阻挡）。</summary>
        void Rasterize(NavGridObstacleMap map);
    }

    /// <summary>
    /// 障碍快照（1 个 ObstacleCellSize 世界单位/格，按层分块）。业务侧通过它写障碍。
    /// </summary>
    /// <remarks>
    /// 每格用一个字节表达：0 = 该层默认（地面层可走，其余层不可走），
    /// 1 = 阻挡，2 及以上 = 区域号 + 2（<see cref="MarkArea"/>，用于墙顶这类"显式可走"的层）。
    /// </remarks>
    public sealed class NavGridObstacleMap
    {
        private readonly NavGridSettings _settings;
        private readonly byte[] _cells;

        public NavGridObstacleMap(NavGridSettings settings, byte[] buffer)
        {
            _settings = settings;
            _cells = buffer;
        }

        public int Side => _settings.ObstacleSide;

        public int LayerCount => _settings.Layers;

        /// <summary>单层的快照格数。</summary>
        public int LayerCellCount => _settings.ObstacleSide * _settings.ObstacleSide;

        public void Clear()
        {
            System.Array.Clear(_cells, 0, _cells.Length);
        }

        #region 地面层

        public void MarkCell(int x, int y)
        {
            MarkCell(0, x, y);
        }

        /// <summary>按世界坐标矩形标记（含左下、不含右上）。</summary>
        public void MarkRect(float minX, float minY, float maxX, float maxY)
        {
            MarkRect(0, minX, minY, maxX, maxY);
        }

        #endregion

        #region 指定层

        public void MarkCell(int layer, int x, int y)
        {
            if (layer < 0 || layer >= LayerCount || x < 0 || x >= Side || y < 0 || y >= Side)
            {
                return;
            }

            _cells[layer * LayerCellCount + x * Side + y] = 1;
        }

        public void MarkRect(int layer, float minX, float minY, float maxX, float maxY)
        {
            if (maxX <= minX || maxY <= minY)
            {
                return;
            }

            int x0 = ToCell(minX);
            int x1 = ToCell(maxX - 0.0001f);
            int y0 = ToCell(minY);
            int y1 = ToCell(maxY - 0.0001f);

            for (int x = x0; x <= x1; x++)
            {
                for (int y = y0; y <= y1; y++)
                {
                    MarkCell(layer, x, y);
                }
            }
        }

        #endregion

        #region 区域号

        /// <summary>
        /// 按区域号标记（可用于非地面层显式声明可走区域，如墙顶）。
        /// 与 <see cref="MarkCell(int,int,int)"/> 互斥：后写的覆盖先写的。
        /// </summary>
        public void MarkArea(int layer, int x, int y, byte area)
        {
            if (layer < 0 || layer >= LayerCount || x < 0 || x >= Side || y < 0 || y >= Side ||
                area > byte.MaxValue - 2)
            {
                return;
            }

            _cells[layer * LayerCellCount + x * Side + y] = (byte)(area + 2);
        }

        /// <summary>按世界坐标矩形标记区域号（含左下、不含右上）。</summary>
        public void MarkAreaRect(int layer, float minX, float minY, float maxX, float maxY, byte area)
        {
            if (maxX <= minX || maxY <= minY)
            {
                return;
            }

            int x0 = ToCell(minX);
            int x1 = ToCell(maxX - 0.0001f);
            int y0 = ToCell(minY);
            int y1 = ToCell(maxY - 0.0001f);

            for (int x = x0; x <= x1; x++)
            {
                for (int y = y0; y <= y1; y++)
                {
                    MarkArea(layer, x, y, area);
                }
            }
        }

        #endregion

        #region 网格

        /// <summary>按 Mesh 的三角形把覆盖到的格标为阻挡（用格心做内点判定，按 XY 投影）。</summary>
        public void MarkMesh(Mesh mesh, int layer = 0)
        {
            MarkMesh(mesh, Matrix4x4.identity, layer);
        }

        /// <summary>按 Mesh 的三角形把覆盖到的格标为阻挡（用格心做内点判定，按 XY 投影）。</summary>
        public void MarkMesh(Mesh mesh, Matrix4x4 matrix, int layer = 0)
        {
            if (mesh == null || layer < 0 || layer >= LayerCount)
            {
                return;
            }

            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            if (vertices == null || triangles == null || triangles.Length < 3)
            {
                return;
            }

            float size = Mathf.Max(_settings.CellSize, _settings.ObstacleCellSize);

            for (int t = 0; t + 2 < triangles.Length; t += 3)
            {
                Vector3 a = matrix.MultiplyPoint3x4(vertices[triangles[t]]);
                Vector3 b = matrix.MultiplyPoint3x4(vertices[triangles[t + 1]]);
                Vector3 c = matrix.MultiplyPoint3x4(vertices[triangles[t + 2]]);

                float minX = Mathf.Min(a.x, Mathf.Min(b.x, c.x));
                float maxX = Mathf.Max(a.x, Mathf.Max(b.x, c.x));
                float minY = Mathf.Min(a.y, Mathf.Min(b.y, c.y));
                float maxY = Mathf.Max(a.y, Mathf.Max(b.y, c.y));

                int x0 = ToCell(minX);
                int x1 = ToCell(maxX);
                int y0 = ToCell(minY);
                int y1 = ToCell(maxY);

                for (int x = x0; x <= x1; x++)
                {
                    float px = (x + 0.5f) * size - _settings.WorldHalf;

                    for (int y = y0; y <= y1; y++)
                    {
                        float py = (y + 0.5f) * size - _settings.WorldHalf;
                        if (ContainsPoint(a.x, a.y, b.x, b.y, c.x, c.y, px, py))
                        {
                            MarkCell(layer, x, y);
                        }
                    }
                }
            }
        }

        /// <summary>二维重心坐标内点判定（含边界）。</summary>
        private static bool ContainsPoint(float ax, float ay, float bx, float by, float cx, float cy, float px, float py)
        {
            float d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by);
            float d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy);
            float d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay);

            bool hasNegative = d1 < 0f || d2 < 0f || d3 < 0f;
            bool hasPositive = d1 > 0f || d2 > 0f || d3 > 0f;
            return !(hasNegative && hasPositive);
        }

        #endregion

        private int ToCell(float world)
        {
            float half = _settings.WorldHalf;
            float size = Mathf.Max(_settings.CellSize, _settings.ObstacleCellSize);
            return Mathf.Clamp(Mathf.FloorToInt((world + half) / size), 0, Side - 1);
        }
    }
}
