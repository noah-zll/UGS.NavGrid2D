using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 网格寻路的全部可调参数。所有尺寸都以「世界单位」表达，内部再换算成格。
    /// </summary>
    /// <remarks>
    /// 一个 <see cref="NavGridSystem"/> 持有一份设置；框架本身不假设地图大小、格距或 World 原点。
    /// </remarks>
    [System.Serializable]
    public sealed class NavGridSettings
    {
        [Header("网格")]
        [Tooltip("世界半径：世界坐标范围是 -WorldHalf ~ +WorldHalf")]
        public float WorldHalf = 200f;

        [Tooltip("寻路格边长（世界单位）。1.0 = 400×400 格/层；0.5 = 800×800，精度更高但 A* 慢约 4 倍")]
        public float CellSize = 1f;

        [Tooltip("障碍快照格边长（世界单位），必须 >= CellSize")]
        public float ObstacleCellSize = 1f;

        [Header("层")]
        [Tooltip("层数：0 = 地面，1 = 墙顶…（单层地图设为 1）")]
        public int LayerCount = 1;

        [Header("通行")]
        [Tooltip("clearance 上限，单位 0.25 世界单位（60 = 15 世界单位）")]
        public byte OpenClearance = 60;

        [Tooltip("连通分量计算使用的 clearance 阈值（2 = 0.5 世界单位）")]
        public byte RegionClearanceNeed = 2;

        [Tooltip("目标不可达时，向外搜索最近可通行格的最大半径（世界单位）")]
        public float NearestSearchRadius = 3f;

        [Header("寻路")]
        [Tooltip("单次 A* 的展开上限")]
        public int MaxExpansions = 12000;

        [Tooltip("单次 A* 的耗时上限（毫秒），0 表示不限。长绕行靠它兜底，避免单帧卡顿")]
        public float MaxPathMilliseconds = 3f;

        [Tooltip("启发值权重：1 = 最优路径；略大于 1 可大幅减少同一代价格的展开量")]
        public float HeuristicWeight = 1.02f;

        [Tooltip("每帧最多处理多少次寻路请求")]
        public int PathRequestsPerFrame = 6;

        [Tooltip("每帧寻路的时间预算（毫秒），0 表示只按次数限制")]
        public float PathTimeBudgetMs = 1.5f;

        [Tooltip("路径缓存按人群变化失效的最小间隔（秒）")]
        public float PathCacheCrowdInterval = 0.5f;

        [Tooltip("路径缓存条数上限")]
        public int PathCacheSize = 512;

        [Header("后台寻路")]
        [Tooltip("寻路放到后台线程：主线程不再被长搜索卡帧，代价是路径晚 1~2 帧回来")]
        public bool AsyncPathfinding = true;

        [Tooltip("后台寻路线程数（每个线程约 2MB scratch，不要开太多）")]
        public int PathWorkerThreads = 2;

        [Tooltip("同时在算的寻路任务上限，超出的请求挂起到下一帧")]
        public int MaxConcurrentPathJobs = 8;

        [Header("代理")]
        [Tooltip("单位要静止这么久才被当作寻路墙（秒）")]
        public float StaticBlockDelay = 0.5f;

        [Tooltip("分离死区：重叠小于该值不纠正")]
        public float OverlapDeadZone = 0.1f;

        [Tooltip("最优候选方向得分低于该值就停下等待，避免绕着人群转圈")]
        public float StopScoreThreshold = 0.3f;

        [Tooltip("路点切换阈值（世界单位）")]
        public float WaypointReach = 0.5f;

        [Tooltip("到达终点判定（世界单位）")]
        public float FinalReach = 0.14f;

        [Tooltip("目标不变时的最短重算间隔（秒）")]
        public float RepathInterval = 2f;

        [Tooltip("目标漂移容差（世界单位）：小于该值不重算，避免路线来回翻转")]
        public float TargetTolerance = 2f;

        // ---- 派生量 ----

        public int Side => Mathf.Max(1, Mathf.RoundToInt(WorldHalf * 2f / Mathf.Max(0.01f, CellSize)));

        public int CellCount => Side * Side;

        /// <summary>层数（至少 1）。</summary>
        public int Layers => Mathf.Max(1, LayerCount);

        /// <summary>所有层加起来的格数。</summary>
        public int TotalCells => CellCount * Layers;

        /// <summary>每世界单位占多少格。</summary>
        public float CellPerUnit => 1f / Mathf.Max(0.01f, CellSize);

        public int ObstacleSide =>
            Mathf.Max(1, Mathf.RoundToInt(WorldHalf * 2f / Mathf.Max(CellSize, ObstacleCellSize)));

        /// <summary>一个障碍快照格跨越多少个寻路格。</summary>
        public int CellsPerObstacleCell =>
            Mathf.Max(1, Mathf.RoundToInt(Mathf.Max(CellSize, ObstacleCellSize) / CellSize));

        /// <summary>增量重建时 clearance 需要外扩的格数（= 最大 clearance 半径）。</summary>
        public int ClearanceMargin => Mathf.CeilToInt(OpenClearance * 0.25f / Mathf.Max(0.01f, CellSize)) + 2;
    }
}
