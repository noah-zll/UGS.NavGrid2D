using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 网格寻路系统：持有网格与代理，对外提供构建、路径查询、可达性、代理登记与局部避障查询。
    /// </summary>
    /// <remarks>
    /// 一个实例对应一张地图/一个世界。业务侧通过 <see cref="AddObstacleSource"/> 提供静态障碍，
    /// 通过 <see cref="CreateAgent"/> 登记移动单位；框架本身不依赖任何具体游戏逻辑。
    /// </remarks>
    public sealed class NavGridSystem
    {
        private struct PathKey : System.IEquatable<PathKey>
        {
            public int Start;
            public int Goal;
            public int StartLayer;
            public int GoalLayer;
            public int Mask;
            public int RadiusBucket;
            public int Version;
            public int PortalVersion;
            public int Crowd;

            public bool Equals(PathKey other)
            {
                return Start == other.Start && Goal == other.Goal &&
                       StartLayer == other.StartLayer && GoalLayer == other.GoalLayer &&
                       Mask == other.Mask && RadiusBucket == other.RadiusBucket &&
                       Version == other.Version && PortalVersion == other.PortalVersion && Crowd == other.Crowd;
            }

            public override bool Equals(object obj)
            {
                return obj is PathKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = Start;
                    hash = hash * 397 ^ Goal;
                    hash = hash * 397 ^ StartLayer;
                    hash = hash * 397 ^ GoalLayer;
                    hash = hash * 397 ^ Mask;
                    hash = hash * 397 ^ RadiusBucket;
                    hash = hash * 397 ^ Version;
                    hash = hash * 397 ^ PortalVersion;
                    hash = hash * 397 ^ Crowd;
                    return hash;
                }
            }
        }

        private sealed class CachedPath
        {
            public NavPathStatus Status;
            public NavPathPoint[] Corners;
            public int Expansions;
            public bool ReachedGoal;
            public bool Truncated;
        }

        /// <summary>空间哈希的桶尺寸（世界单位）。</summary>
        private const float BucketSize = 2f;

        private readonly NavGridSettings _settings;
        private readonly NavGridBuilder _builder;
        private readonly List<INavGridObstacleSource> _sources = new List<INavGridObstacleSource>();
        private readonly Dictionary<PathKey, CachedPath> _pathCache;
        private readonly List<GridAgent> _agents = new List<GridAgent>(256);
        private readonly Dictionary<long, List<GridAgent>> _buckets = new Dictionary<long, List<GridAgent>>(1024);
        private readonly List<List<GridAgent>> _bucketPool = new List<List<GridAgent>>(1024);
        private readonly float[] _areaCosts = new float[32];

        private int _bucketFrame = -1;
        private int _bucketPoolUsed;
        private int _pathBudgetFrame = -1;
        private int _pathBudgetLeft;
        private long _pathBudgetSpentTicks;
        private float _lastCrowdCacheClear;
        private int _nextAgentId;

        private NavPathWorker _pathWorker;
        private bool _rebuildPending;
        private NavPortalSnapshot _portalSnapshot;
        private int _portalSnapshotVersion = -1;

        public NavGridSystem(NavGridSettings settings = null)
        {
            _settings = settings ?? new NavGridSettings();
            _builder = new NavGridBuilder(_settings);
            _pathCache = new Dictionary<PathKey, CachedPath>(_settings.PathCacheSize);

            for (int i = 0; i < _areaCosts.Length; i++)
            {
                _areaCosts[i] = 1f;
            }
        }

        public NavGridSettings Settings => _settings;

        public NavGrid Grid { get; private set; }

        /// <summary>门户图：层之间唯一的连通方式。</summary>
        public NavPortalGraph Portals { get; } = new NavPortalGraph();

        public int Version => Grid != null ? Grid.Version : 0;

        public int AgentCount => _agents.Count;

        /// <summary>网格已构建完成，才可以把移动交给网格代理。</summary>
        public bool IsReady => Grid != null && Grid.Version > 0;

        /// <summary>最近一次构建耗时（毫秒），供业务侧打日志。</summary>
        public double LastRebuildMilliseconds { get; private set; }

        /// <summary>最近一次寻路的展开次数（诊断用）。</summary>
        public int LastPathExpansions { get; private set; }

        /// <summary>最近一次寻路是否真的走到目标；false 表示被展开上限/时间上限截断。</summary>
        public bool LastPathReachedGoal { get; private set; }

        /// <summary>最近一次寻路是否被展开上限截断（false 且未到达目标 = 目标确实不可达，再补段也没用）。</summary>
        public bool LastPathTruncated { get; private set; }

        /// <summary>最近一次寻路耗时（毫秒）；命中缓存时为 0。</summary>
        public float LastPathMilliseconds { get; private set; }

        public IReadOnlyList<INavGridObstacleSource> ObstacleSources => _sources;

        #region 障碍源

        public void AddObstacleSource(INavGridObstacleSource source)
        {
            if (source != null && !_sources.Contains(source))
            {
                _sources.Add(source);
            }
        }

        public void RemoveObstacleSource(INavGridObstacleSource source)
        {
            if (source != null)
            {
                _sources.Remove(source);
            }
        }

        #endregion

        #region 构建与清理

        /// <summary>重建网格，返回本次改动的障碍格数量（0 = 没有任何变化，网格版本不变）。</summary>
        public int Rebuild()
        {
            // 后台寻路正在读 Area/Clearance/Cost：此时改会让它看到"半新半旧"的网格，
            // 可能算出穿墙路径。这里直接延后到任务全部结束（<see cref="TryRunPendingRebuild"/> 会补跑）。
            if (_pathWorker != null && _pathWorker.InFlight > 0)
            {
                _rebuildPending = true;
                return 0;
            }

            _rebuildPending = false;

            Grid ??= new NavGrid(_settings);

            long startTimestamp = System.Diagnostics.Stopwatch.GetTimestamp();
            int changedCells = _builder.Build(Grid, _sources);
            LastRebuildMilliseconds = (System.Diagnostics.Stopwatch.GetTimestamp() - startTimestamp) * 1000.0 /
                                      System.Diagnostics.Stopwatch.Frequency;

            _pathCache.Clear();
            return changedCells;
        }

        /// <summary>
        /// 延后重建的补跑入口。每帧（避障查询、寻路请求）都会经过这里，
        /// 框架没有统一的 Update，所以挂在热点路径上。
        /// </summary>
        private void TryRunPendingRebuild()
        {
            if (_rebuildPending && (_pathWorker == null || _pathWorker.InFlight == 0))
            {
                Rebuild();
            }
        }

        /// <summary>清空网格与所有代理登记（换图/关闭地图时调用）。</summary>
        public void Clear()
        {
            Grid = null;
            Portals.Clear();
            _builder.Reset();
            _pathCache.Clear();
            _agents.Clear();
            _buckets.Clear();
            _bucketFrame = -1;
            _pathBudgetFrame = -1;
            _pathBudgetSpentTicks = 0;
            _lastCrowdCacheClear = 0f;
            _rebuildPending = false;
            _portalSnapshot = null;
            _portalSnapshotVersion = -1;

            // 后台任务读的是旧网格，直接丢掉；线程池按需重建
            if (_pathWorker != null)
            {
                _pathWorker.Shutdown();
                _pathWorker = null;
            }
        }

        #endregion

        #region 查询

        /// <summary>是否把寻路放到后台线程（<see cref="NavGridSettings.AsyncPathfinding"/>）。</summary>
        public bool UseAsyncPathfinding => _settings.AsyncPathfinding;

        /// <summary>已提交、尚未算完的后台寻路任务数。</summary>
        public int PendingPathJobs => _pathWorker != null ? _pathWorker.InFlight : 0;

        private NavPathWorker Worker
        {
            get
            {
                if (_pathWorker == null)
                {
                    _pathWorker = new NavPathWorker(_settings.PathWorkerThreads);
                }

                return _pathWorker;
            }
        }

        /// <summary>
        /// 提交一次后台寻路，立即返回。结果在 <see cref="NavPathJob.IsDone"/> 之后可读，
        /// 只允许提交方（代理）消费。
        /// </summary>
        /// <remarks>
        /// 整条管线（正常 → 忽略人群 → 强启发 → 补段）都在工作线程里跑完，
        /// 主线程只负责提交和读结果，所以长绕行不会卡帧，代价是路径晚 1~2 帧回来。
        /// </remarks>
        public NavPathJob RequestPath(Vector2 start, int startLayer, Vector2 goal, int goalLayer, int areaMask,
            float radius, bool ignoreCrowd, int maxExpansions, float maxMilliseconds,
            bool relaxCrowdOnFail, bool greedyFallback, int greedyExpansions, float greedyMilliseconds,
            float greedyWeight, int maxStitchSegments)
        {
            if (Grid == null)
            {
                return null;
            }

            TryRunPendingRebuild();

            NavPathJob job = new NavPathJob
            {
                Grid = Grid,
                Portals = PortalQuery,
                StartLayer = startLayer,
                GoalLayer = goalLayer,
                Start = start,
                Goal = goal,
                AreaMask = areaMask,
                Radius = radius,
                IgnoreCrowd = ignoreCrowd,
                MaxExpansions = maxExpansions,
                MaxMilliseconds = maxMilliseconds,
                HeuristicWeight = 0f,
                RelaxCrowdOnFail = relaxCrowdOnFail,
                GreedyFallback = greedyFallback,
                GreedyExpansions = greedyExpansions,
                GreedyMilliseconds = greedyMilliseconds,
                GreedyWeight = greedyWeight,
                MaxStitchSegments = maxStitchSegments,
                FinalReachSqr = _settings.FinalReach * _settings.FinalReach,
            };

            Worker.Submit(job);
            return job;
        }

        /// <summary>
        /// 门户的不可变快照（按 <see cref="NavPortalGraph.Version"/> 缓存）。
        /// 后台线程读的是它，主线程改门户图不会影响正在跑的搜索。
        /// </summary>
        public INavPortalQuery PortalQuery
        {
            get
            {
                if (_portalSnapshot == null || _portalSnapshotVersion != Portals.Version)
                {
                    _portalSnapshot = new NavPortalSnapshot(Portals);
                    _portalSnapshotVersion = Portals.Version;
                }

                return _portalSnapshot;
            }
        }

        public NavPathStatus CalculatePath(Vector2 start, Vector2 goal, int areaMask, float radius,
            List<NavPathPoint> corners, bool ignoreCrowd)
        {
            return CalculatePath(start, 0, goal, 0, areaMask, radius, corners, ignoreCrowd);
        }

        public NavPathStatus CalculatePath(Vector2 start, int startLayer, Vector2 goal, int goalLayer, int areaMask,
            float radius, List<NavPathPoint> corners, bool ignoreCrowd, int maxExpansions = 0,
            float maxMilliseconds = 0f, float heuristicWeight = 0f)
        {
            corners.Clear();

            NavGrid grid = Grid;
            if (grid == null)
            {
                return NavPathStatus.Invalid;
            }

            TryRunPendingRebuild();

            // 自定义预算的请求不进缓存：同一个起终点在小预算下可能是 Partial，不能复用
            bool useCache = maxExpansions <= 0 && maxMilliseconds <= 0f && heuristicWeight <= 0f;

            PathKey key = new PathKey
            {
                Start = grid.CellIndex(start),
                Goal = grid.CellIndex(goal),
                StartLayer = startLayer,
                GoalLayer = goalLayer,
                Mask = areaMask,
                RadiusBucket = NavGrid.ClearanceNeed(radius),
                Version = grid.Version,
                PortalVersion = Portals.Version,
                Crowd = ignoreCrowd ? 1 : 0,
            };

            if (useCache && _pathCache.TryGetValue(key, out CachedPath cached) && cached.Corners != null)
            {
                corners.AddRange(cached.Corners);
                LastPathExpansions = cached.Expansions;
                LastPathReachedGoal = cached.ReachedGoal;
                LastPathTruncated = cached.Truncated;
                LastPathMilliseconds = 0f;
                return cached.Status;
            }

            long startTimestamp = System.Diagnostics.Stopwatch.GetTimestamp();

            NavPathStatus status = NavPathfinder.FindPath(grid, PortalQuery, startLayer, start, goalLayer, goal,
                areaMask, radius, corners, ignoreCrowd, maxExpansions, maxMilliseconds, heuristicWeight);

            LastPathExpansions = NavPathfinder.LastExpansions;
            LastPathReachedGoal = NavPathfinder.LastReachedGoal;
            LastPathTruncated = NavPathfinder.LastTruncated;
            LastPathMilliseconds = NavPathfinder.LastMilliseconds;

            _pathBudgetSpentTicks += System.Diagnostics.Stopwatch.GetTimestamp() - startTimestamp;

            if (useCache)
            {
                if (_pathCache.Count >= _settings.PathCacheSize)
                {
                    _pathCache.Clear();
                }

                _pathCache[key] = new CachedPath
                {
                    Status = status,
                    Corners = corners.ToArray(),
                    Expansions = LastPathExpansions,
                    ReachedGoal = LastPathReachedGoal,
                    Truncated = LastPathTruncated,
                };
            }

            return status;
        }

        /// <summary>
        /// 目标点是否可达。与 <c>NavMeshAgent.CalculatePath</c> 的用途一致，但走连通分量，代价很低。
        /// 只对同层有效；跨层可达性由门户决定，需走 <see cref="CalculatePath"/>。
        /// </summary>
        public bool IsReachable(Vector2 from, Vector2 to, int areaMask, float radius)
        {
            return IsReachable(0, from, to, areaMask, radius);
        }

        public bool IsReachable(int layer, Vector2 from, Vector2 to, int areaMask, float radius)
        {
            NavGrid grid = Grid;
            if (grid == null)
            {
                return false;
            }

            Vector2 goal = to;
            if (!grid.IsPassable(layer, to, areaMask, radius) &&
                !grid.SamplePosition(layer, to, areaMask, radius, radius + 1.5f, out goal))
            {
                return false;
            }

            return grid.IsReachable(layer, from, goal, radius);
        }

        public Vector2 SamplePosition(Vector2 position, int areaMask, float radius, float maxRadius)
        {
            return SamplePosition(0, position, areaMask, radius, maxRadius);
        }

        public Vector2 SamplePosition(int layer, Vector2 position, int areaMask, float radius, float maxRadius)
        {
            NavGrid grid = Grid;
            if (grid == null)
            {
                return position;
            }

            return grid.SamplePosition(layer, position, areaMask, radius, maxRadius, out Vector2 result)
                ? result
                : position;
        }

        public int GetAreaIndex(Vector2 position)
        {
            return GetAreaIndex(0, position);
        }

        public int GetAreaIndex(int layer, Vector2 position)
        {
            return Grid != null ? Grid.GetAreaIndex(layer, position) : -1;
        }

        public bool IsPassable(Vector2 world, int areaMask, float radius)
        {
            return IsPassable(0, world, areaMask, radius);
        }

        public bool IsPassable(int layer, Vector2 world, int areaMask, float radius)
        {
            NavGrid grid = Grid;
            return grid != null && grid.IsPassable(layer, world, areaMask, radius);
        }

        public float GetAreaCost(int area)
        {
            return area >= 0 && area < _areaCosts.Length ? _areaCosts[area] : 1f;
        }

        public void SetAreaCost(int area, float cost)
        {
            if (area >= 0 && area < _areaCosts.Length)
            {
                _areaCosts[area] = Mathf.Max(0.01f, cost);
            }
        }

        /// <summary>
        /// 本帧还能不能新建一次寻路请求。
        /// 同时受次数与时间预算限制：几十个单位同帧重算时，避免单帧出现一次十几毫秒的尖峰。
        /// </summary>
        public bool TryConsumePathBudget()
        {
            if (_pathBudgetFrame != Time.frameCount)
            {
                _pathBudgetFrame = Time.frameCount;
                _pathBudgetLeft = Mathf.Max(1, _settings.PathRequestsPerFrame);
                _pathBudgetSpentTicks = 0;
            }

            if (_pathBudgetLeft <= 0)
            {
                return false;
            }

            if (_settings.PathTimeBudgetMs > 0f && SpentMilliseconds() >= _settings.PathTimeBudgetMs)
            {
                return false;
            }

            _pathBudgetLeft--;
            return true;
        }

        private double SpentMilliseconds()
        {
            return _pathBudgetSpentTicks * 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        }

        #endregion

        #region 代理登记与空间哈希

        /// <summary>创建并登记一个网格代理。</summary>
        public GridAgent CreateAgent(int id, Transform transform, float radius)
        {
            GridAgent agent = new GridAgent(this, id, transform, radius);
            Register(agent);
            return agent;
        }

        /// <summary>创建并登记一个网格代理（Id 由系统分配）。</summary>
        public GridAgent CreateAgent(Transform transform, float radius)
        {
            return CreateAgent(_nextAgentId++, transform, radius);
        }

        public void Register(GridAgent agent)
        {
            if (agent == null || _agents.Contains(agent))
            {
                return;
            }

            _agents.Add(agent);
            _bucketFrame = -1;
        }

        public void Unregister(GridAgent agent)
        {
            if (agent == null)
            {
                return;
            }

            _agents.Remove(agent);
            _bucketFrame = -1;
        }

        /// <summary>把当前登记的代理拷出来（调试可视化用）。</summary>
        public void CopyAgents(List<GridAgent> results)
        {
            results.Clear();
            results.AddRange(_agents);
        }

        /// <summary>取附近代理（用于局部避障）。不含 <paramref name="self"/>。</summary>
        public void QueryNeighbors(GridAgent self, float radius, List<GridAgent> results)
        {
            results.Clear();
            if (self == null || _agents.Count == 0)
            {
                return;
            }

            EnsureBuckets();

            Vector2 position = self.Position;
            float half = _settings.WorldHalf;
            int minX = Mathf.FloorToInt((position.x - radius + half) / BucketSize);
            int maxX = Mathf.FloorToInt((position.x + radius + half) / BucketSize);
            int minY = Mathf.FloorToInt((position.y - radius + half) / BucketSize);
            int maxY = Mathf.FloorToInt((position.y + radius + half) / BucketSize);

            float sqrRadius = radius * radius;

            for (int x = minX; x <= maxX; x++)
            {
                for (int y = minY; y <= maxY; y++)
                {
                    if (!_buckets.TryGetValue(MakeBucketKey(x, y), out List<GridAgent> bucket))
                    {
                        continue;
                    }

                    for (int i = 0; i < bucket.Count; i++)
                    {
                        GridAgent other = bucket[i];
                        if (other == self || !other.Enabled)
                        {
                            continue;
                        }

                        if ((other.Position - position).sqrMagnitude <= sqrRadius)
                        {
                            results.Add(other);
                        }
                    }
                }
            }
        }

        /// <summary>每帧第一次查询时重建空间哈希，并刷新人群密度/静止阻挡。</summary>
        private void EnsureBuckets()
        {
            // 框架没有统一 Update，这里近似每帧一次：顺带补跑被后台寻路挡下的网格重建
            TryRunPendingRebuild();

            if (_bucketFrame == Time.frameCount)
            {
                return;
            }

            _bucketFrame = Time.frameCount;
            _bucketPoolUsed = 0;
            _buckets.Clear();

            NavGrid grid = Grid;
            grid?.ClearCrowd();

            float half = _settings.WorldHalf;

            for (int i = 0; i < _agents.Count; i++)
            {
                GridAgent agent = _agents[i];
                Vector2 position = agent.Position;

                if (grid != null && agent.Enabled)
                {
                    // 人群密度：用于调试热力
                    grid.AddDensity(position);

                    // 只有"站够了时间还没动"的单位才写寻路阻挡：
                    // 站着的必须绕，走着的互相不挡；加延迟避免状态逐帧翻转引起全图改路
                    if (agent.StaticTime >= _settings.StaticBlockDelay)
                    {
                        grid.AddStaticBlock(position);
                    }
                }

                long key = MakeBucketKey(
                    Mathf.FloorToInt((position.x + half) / BucketSize),
                    Mathf.FloorToInt((position.y + half) / BucketSize));

                if (!_buckets.TryGetValue(key, out List<GridAgent> bucket))
                {
                    bucket = GetBucketFromPool();
                    _buckets[key] = bucket;
                }

                bucket.Add(agent);
            }

            // 人群每帧都在变，但路径本身每 2s 就会重算一次，没必要逐帧清缓存：
            // 逐帧清等于缓存永远不命中，50v50 时会白白多跑大量 A*。
            float now = Time.time;
            if (now - _lastCrowdCacheClear >= _settings.PathCacheCrowdInterval)
            {
                _pathCache.Clear();
                _lastCrowdCacheClear = now;
            }
        }

        private List<GridAgent> GetBucketFromPool()
        {
            if (_bucketPoolUsed < _bucketPool.Count)
            {
                List<GridAgent> reused = _bucketPool[_bucketPoolUsed++];
                reused.Clear();
                return reused;
            }

            List<GridAgent> created = new List<GridAgent>(8);
            _bucketPool.Add(created);
            _bucketPoolUsed++;
            return created;
        }

        private static long MakeBucketKey(int x, int y)
        {
            return ((long)x << 32) ^ (uint)y;
        }

        #endregion
    }
}
