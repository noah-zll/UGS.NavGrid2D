namespace UGS.NavGrid2D
{
    /// <summary>路径状态，语义与 <c>NavMeshPathStatus</c> 对齐。</summary>
    public enum NavPathStatus
    {
        /// <summary>起点到终点完整连通。</summary>
        Complete = 0,

        /// <summary>终点不可达，返回的是离终点最近的可达点。</summary>
        Partial = 1,

        /// <summary>连起点附近的可行走格都找不到。</summary>
        Invalid = 2,
    }
}
