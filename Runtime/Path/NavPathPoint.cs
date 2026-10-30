using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>路径点：世界坐标 + 所在层。跨层时相邻两点的层号会不同，这段就是门户穿越。</summary>
    public struct NavPathPoint
    {
        public Vector2 Position;
        public int Layer;

        public NavPathPoint(Vector2 position, int layer)
        {
            Position = position;
            Layer = layer;
        }
    }
}
