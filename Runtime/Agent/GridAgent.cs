using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 网格代理：替代 <c>NavMeshAgent</c> 承担"按路径移动"这一层。
    /// </summary>
    /// <remarks>
    /// 由宿主每帧调用 <see cref="Tick"/> 推进位置；只负责沿路径走 + 局部避障，
    /// 寻路交给 <see cref="NavGridSystem.CalculatePath"/>。
    /// </remarks>
    public sealed class GridAgent
    {
        private readonly NavGridSystem _system;
        private readonly Transform _transform;
        private readonly List<NavPathPoint> _corners = new List<NavPathPoint>(32);
        private readonly List<NavPathPoint> _calculateBuffer = new List<NavPathPoint>(32);

        private readonly float _waypointReachSqr;
        private readonly float _finalReachSqr;
        private readonly float _repathInterval;
        private readonly float _repathJitter;
        private readonly float _targetToleranceSqr;

        private Vector3[] _cornerCache;
        private int _cornerIndex;
        private float _stuckTimer;
        private float _nextRepathTime;
        private bool _hasCompletePath;

        /// <summary>上一次真正参与寻路的目标（Partial 路径也要靠它判断"目标没变、可以复用"）。</summary>
        private Vector2 _requestTarget;

        /// <summary>
        /// 预算内搜不完时的重试启发权重：强启发会贴着障碍朝目标推进，展开量小得多，
        /// 代价是路径可能略长。只在正常搜索失败后才用。
        /// </summary>
        private const float FallbackHeuristicWeight = 2.5f;

        /// <summary>
        /// 强启发重试的预算（展开数/毫秒）：正常搜索失败才走这条，允许比常规大得多。
        /// 编辑器里每个节点约 0.8µs，20ms 大约 2 万节点，刚好够绕完一段长墙。
        /// </summary>
        private const int FallbackPathExpansions = 60000;
        private const float FallbackPathMilliseconds = 20f;

        /// <summary>路径被截断时最多再补几段（每段一次 A*）。</summary>
        private const int MaxStitchSegments = 1;

        private readonly List<NavPathPoint> _stitchBuffer = new List<NavPathPoint>(256);

        /// <summary>上一次寻路返回的状态，用于"目标没变、还没到重算时间"时直接复用。</summary>
        private NavPathStatus _lastStatus = NavPathStatus.Invalid;

        // 预算用尽时先记下目标，下一帧补算：直接返回失败会让上层以为"走不过去"
        private bool _hasPendingDestination;
        private Vector2 _pendingTarget;
        private int _pendingLayer;

        /// <summary>后台寻路开关（构造时从设置里缓存）。</summary>
        private readonly bool _asyncPathfinding;

        /// <summary>正在后台计算的任务；结果到了在 <see cref="ApplyFinishedPathJob"/> 里落地。</summary>
        private NavPathJob _asyncJob;
        private Vector2 _asyncTarget;

        public readonly int Id;

        public float Radius;
        public float Speed;
        public float Acceleration = 180f;
        public int AreaMask = 1;

        /// <summary>代理所在层（0 = 地面，1 = 墙顶…）。寻路起点用该层。</summary>
        public int Layer;

        public int AvoidancePriority = 99;
        public bool AvoidanceEnabled = true;
        public bool Enabled = true;
        public bool IsStopped;

        /// <summary>
        /// true 时寻路忽略人群代价（阵型整体推进用），避免把自己的队友当障碍绕。
        /// </summary>
        public bool IgnoreCrowd;

        /// <summary>
        /// 单次寻路的展开上限覆盖（0 = 用全局设置）。玩家手动指令可以给更大的预算。
        /// </summary>
        public int MaxPathExpansions;

        /// <summary>
        /// 单次寻路的耗时上限覆盖（毫秒，0 = 用全局设置）。长绕行靠它兜底。
        /// </summary>
        public float MaxPathMilliseconds;

        /// <summary>最近一次寻路实际展开的节点数（诊断用）。</summary>
        public int LastPathExpansions { get; private set; }

        /// <summary>最近一次寻路耗时（毫秒，诊断用）。</summary>
        public float LastPathMilliseconds { get; private set; }

        /// <summary>最近一次寻路是否被展开/时间上限截断（false = 已经搜完整个可达区域，目标确实到不了）。</summary>
        public bool LastPathTruncated { get; private set; }

        public GridAgent(NavGridSystem system, int id, Transform transform, float radius)
        {
            _system = system;
            Id = id;
            _transform = transform;
            Radius = radius;
            PathEndPosition = transform != null ? (Vector2)transform.position : Vector2.zero;

            NavGridSettings settings = system != null ? system.Settings : new NavGridSettings();
            _waypointReachSqr = settings.WaypointReach * settings.WaypointReach;
            _finalReachSqr = settings.FinalReach * settings.FinalReach;
            _repathInterval = settings.RepathInterval;
            _repathJitter = (id % 10) * 0.1f;
            _targetToleranceSqr = settings.TargetTolerance * settings.TargetTolerance;
            _asyncPathfinding = settings.AsyncPathfinding;
        }

        public Vector2 Velocity { get; private set; }

        /// <summary>当前路径的终点（与 <c>NavMeshAgent.pathEndPosition</c> 语义一致）。</summary>
        public Vector2 PathEndPosition { get; private set; }

        /// <summary>当前框架不做门户穿越，恒为 false。</summary>
        public bool IsOnPortal => false;

        public bool HasPath => _cornerIndex < _corners.Count;

        /// <summary>有路径但持续走不动的时间（秒），供上层做纠偏。</summary>
        public float StuckTime => _stuckTimer;

        /// <summary>连续静止的时间（秒），超过阈值才被视为寻路墙。</summary>
        public float StaticTime { get; private set; }

        public Vector2 Position => _transform != null ? (Vector2)_transform.position : Vector2.zero;

        /// <summary>飞行等场景把 areaMask 设为 ~0，此时直线移动、不做寻路与避障。</summary>
        public bool FreeMove => AreaMask == ~0;

        public Vector3[] PathCorners
        {
            get
            {
                int count = _corners.Count - _cornerIndex;
                int total = count + 1;

                if (_cornerCache == null || _cornerCache.Length != total)
                {
                    _cornerCache = new Vector3[total];
                }

                Vector2 position = Position;
                float z = _transform != null ? _transform.position.z : 0f;
                _cornerCache[0] = new Vector3(position.x, position.y, z);

                for (int i = 0; i < count; i++)
                {
                    Vector2 corner = _corners[_cornerIndex + i].Position;
                    _cornerCache[i + 1] = new Vector3(corner.x, corner.y, z);
                }

                return _cornerCache;
            }
        }

        /// <summary>
        /// 请求移动到目标点。返回 false 表示网格算不出完整路径，调用方应回退到 NavMesh。
        /// </summary>
        public bool SetDestination(Vector2 target)
        {
            return SetDestinationWithStatus(target) == NavPathStatus.Complete;
        }

        /// <summary>
        /// 请求移动到目标点，并返回路径状态。
        /// Partial 时路径终点会被改成"离目标最近的可达点"。
        /// </summary>
        public NavPathStatus SetDestinationWithStatus(Vector2 target)
        {
            return SetDestinationWithStatus(target, ResolveGoalLayer(target));
        }

        /// <summary>
        /// 目标点落在哪一层：优先当前层，否则取第一个能站住该点的层（墙顶 ↔ 地面）。
        /// </summary>
        private int ResolveGoalLayer(Vector2 target)
        {
            NavGrid grid = _system != null ? _system.Grid : null;
            if (grid == null)
            {
                return Layer;
            }

            if (grid.IsPassable(Layer, target, AreaMask, Radius))
            {
                return Layer;
            }

            for (int layer = 0; layer < grid.LayerCount; layer++)
            {
                if (layer != Layer && grid.IsPassable(layer, target, AreaMask, Radius))
                {
                    return layer;
                }
            }

            return Layer;
        }

        /// <summary>指定目标层（爬梯上墙等）；起点层始终是 <see cref="Layer"/>。</summary>
        public NavPathStatus SetDestinationWithStatus(Vector2 target, int targetLayer)
        {
            if (!Enabled)
            {
                return NavPathStatus.Invalid;
            }

            // 后台结果到了就先落地：上层是按"设置目标"的频率调用的，路径晚一帧回来不影响
            ApplyFinishedPathJob();

            // 已有路径且目标没怎么动时，不再重算：
            // 否则频繁重算会让"绕行/直穿"来回翻转，走到一半就拐回去挤。
            // Partial 路径同样复用（否则每帧重算，路径显示会不停跳动）；到达 Partial 终点后
            // Tick 会 ResetPath，下次调用重新算下一段。
            if (HasPath && Time.time < _nextRepathTime &&
                (target - _requestTarget).sqrMagnitude <= _targetToleranceSqr)
            {
                return _hasCompletePath ? NavPathStatus.Complete : NavPathStatus.Partial;
            }

            // 没有路径（已走到断点或被清空）且目标没变：在重算间隔内直接返回上次结果。
            // 目标不可达时上层每 0.3 秒重发一次请求，不挡的话会不停跑长搜索，表现为持续掉帧。
            if (!HasPath && Time.time < _nextRepathTime &&
                (target - _requestTarget).sqrMagnitude <= _targetToleranceSqr)
            {
                return _lastStatus;
            }

            if (FreeMove)
            {
                SetPathDirect(target);
                _lastStatus = NavPathStatus.Complete;
                return NavPathStatus.Complete;
            }

            if (_system == null || !_system.IsReady)
            {
                return NavPathStatus.Invalid;
            }

            if (_asyncPathfinding)
            {
                return SubmitAsyncPath(target, targetLayer);
            }

            // 每帧 A* 有全局预算：用尽时把目标挂起，下一帧补算，避免上层误判"不可达"而停住
            if (!_system.TryConsumePathBudget())
            {
                if (!HasPath)
                {
                    _hasPendingDestination = true;
                    _pendingTarget = target;
                    _pendingLayer = targetLayer;
                    return NavPathStatus.Partial;
                }

                return NavPathStatus.Complete;
            }

            _requestTarget = target;
            _nextRepathTime = Time.time + _repathInterval + _repathJitter;

            NavPathStatus status = _system.CalculatePath(Position, Layer, target, targetLayer, AreaMask, Radius,
                _calculateBuffer, IgnoreCrowd, MaxPathExpansions, MaxPathMilliseconds);

            // 站着的单位只在起点/终点附近豁免：目标被人群围住时正常寻路会"到不了"，
            // 于是路径停在中途、上层反复重算。这种情况退一步忽略人群再算一次。
            if (status != NavPathStatus.Complete && !IgnoreCrowd)
            {
                NavPathStatus relaxed = _system.CalculatePath(Position, Layer, target, targetLayer, AreaMask, Radius,
                    _calculateBuffer, true, MaxPathExpansions, MaxPathMilliseconds);
                if (relaxed == NavPathStatus.Complete)
                {
                    status = relaxed;
                }
            }

            // 预算内没搜完（长绕行）：换成强启发再搜一次。强启发会贴着障碍朝目标推进，
            // 展开量小得多；它的"最近已展开点"通常也比普通搜索更靠近目标，所以即使仍是 Partial 也采用。
            if (status != NavPathStatus.Complete && _system.LastPathTruncated)
            {
                NavPathStatus greedy = _system.CalculatePath(Position, Layer, target, targetLayer, AreaMask, Radius,
                    _calculateBuffer, true, FallbackPathExpansions, FallbackPathMilliseconds,
                    FallbackHeuristicWeight);
                if (greedy != NavPathStatus.Invalid)
                {
                    status = greedy;
                }
            }

            // 只有"被展开上限截断"才值得补段继续算；
            // 整片区域都搜完还没到目标，说明目标真的不可达，补多少段都是同一个结果
            if (status == NavPathStatus.Partial && _system.LastPathTruncated)
            {
                status = StitchPartialPath(target, targetLayer, status);
            }

            LastPathExpansions = _system.LastPathExpansions;
            LastPathMilliseconds = _system.LastPathMilliseconds;
            LastPathTruncated = _system.LastPathTruncated;

            _hasCompletePath = status == NavPathStatus.Complete;

            if (status == NavPathStatus.Complete)
            {
                SetPath(_calculateBuffer, target);
                _lastStatus = status;
                return status;
            }

            if (status == NavPathStatus.Partial && _calculateBuffer.Count > 0)
            {
                Vector2 nearest = _calculateBuffer[_calculateBuffer.Count - 1].Position;
                SetPath(_calculateBuffer, nearest);
            }

            _lastStatus = status;
            return status;
        }

        /// <summary>
        /// 路径被展开上限/时间上限截断时，从已算出的终点继续算下一段并拼接。
        /// 只补有限段（<see cref="MaxStitchSegments"/>），避免一次请求拖太久；
        /// 上层仍会按 <c>RepathInterval</c> 正常重算，这里只是让这一条路径尽量连到终点。
        /// </summary>
        private NavPathStatus StitchPartialPath(Vector2 target, int targetLayer, NavPathStatus status)
        {
            for (int i = 0; i < MaxStitchSegments && status == NavPathStatus.Partial; i++)
            {
                if (_calculateBuffer.Count == 0)
                {
                    return status;
                }

                NavPathPoint tail = _calculateBuffer[_calculateBuffer.Count - 1];
                if ((tail.Position - target).sqrMagnitude <= _finalReachSqr)
                {
                    return status;
                }

                _stitchBuffer.Clear();
                NavPathStatus next = _system.CalculatePath(tail.Position, tail.Layer, target, targetLayer,
                    AreaMask, Radius, _stitchBuffer, IgnoreCrowd);

                if (_stitchBuffer.Count == 0)
                {
                    return status;
                }

                _calculateBuffer.AddRange(_stitchBuffer);
                status = next;
            }

            return status;
        }

        /// <summary>
        /// 后台寻路：提交一次请求立即返回，结果到了由 <see cref="ApplyFinishedPathJob"/> 落地。
        /// </summary>
        private NavPathStatus SubmitAsyncPath(Vector2 target, int targetLayer)
        {
            // 上一次的请求还在算：等它回来，别推翻（推翻等于白算一次）
            if (_asyncJob != null && !_asyncJob.IsDone)
            {
                return _lastStatus;
            }

            if (_system.PendingPathJobs >= _system.Settings.MaxConcurrentPathJobs)
            {
                // 和主线程版"预算用尽"一致：挂起到下一帧，不要让上层误判成不可达
                _hasPendingDestination = true;
                _pendingTarget = target;
                _pendingLayer = targetLayer;
                return _lastStatus;
            }

            _asyncJob?.Cancel();
            _asyncJob = _system.RequestPath(Position, Layer, target, targetLayer, AreaMask, Radius, IgnoreCrowd,
                MaxPathExpansions, MaxPathMilliseconds, true, true,
                FallbackPathExpansions, FallbackPathMilliseconds, FallbackHeuristicWeight, MaxStitchSegments);

            if (_asyncJob == null)
            {
                return NavPathStatus.Invalid;
            }

            _asyncTarget = target;
            _requestTarget = target;
            _nextRepathTime = Time.time + _repathInterval + _repathJitter;

            // 结果还没回来：先按"已接受"回给上层（返回 Invalid 会让上层当作走不过去而停住）
            return _lastStatus != NavPathStatus.Invalid ? _lastStatus : NavPathStatus.Partial;
        }

        /// <summary>把已完成的后台路径落到代理上（主线程调用）。</summary>
        private void ApplyFinishedPathJob()
        {
            NavPathJob job = _asyncJob;
            if (job == null || !job.IsDone)
            {
                return;
            }

            _asyncJob = null;

            LastPathExpansions = job.Expansions;
            LastPathMilliseconds = job.Milliseconds;
            LastPathTruncated = job.Truncated;

            NavPathStatus status = job.Status;
            _lastStatus = status;
            _hasCompletePath = status == NavPathStatus.Complete;

            if (status == NavPathStatus.Complete)
            {
                SetPath(job.Corners, _asyncTarget);
                return;
            }

            // Partial：走"离目标最近的可达点"，与同步版一致
            if (status == NavPathStatus.Partial && job.Corners.Count > 0)
            {
                SetPath(job.Corners, job.Corners[job.Corners.Count - 1].Position);
            }
        }

        /// <summary>直接给定路径点（不含当前位置，含终点）。</summary>
        public void SetPath(List<NavPathPoint> corners, Vector2 target)
        {
            _corners.Clear();
            for (int i = 0; i < corners.Count; i++)
            {
                _corners.Add(corners[i]);
            }

            _cornerIndex = 0;
            _stuckTimer = 0f;
            PathEndPosition = target;

            if (_corners.Count == 0)
            {
                PathEndPosition = Position;
            }
        }

        private void SetPathDirect(Vector2 target)
        {
            _corners.Clear();
            _corners.Add(new NavPathPoint(target, Layer));
            _cornerIndex = 0;
            _stuckTimer = 0f;
            _hasCompletePath = true;
            PathEndPosition = target;
        }

        /// <summary>是否能通过网格到达目标（不改变当前路径）。</summary>
        public bool CanReach(Vector2 target)
        {
            if (_system == null || !_system.IsReady)
            {
                return false;
            }

            if (FreeMove)
            {
                return true;
            }

            NavPathStatus status = _system.CalculatePath(Position, Layer, target, ResolveGoalLayer(target), AreaMask,
                Radius, _calculateBuffer, IgnoreCrowd);
            return status == NavPathStatus.Complete;
        }

        public void Teleport(Vector2 position)
        {
            if (_transform != null)
            {
                _transform.position = new Vector3(position.x, position.y, _transform.position.z);
            }

            ResetPath();
        }

        public void ResetPath()
        {
            // 丢弃还在算的后台任务，否则它落地后会把刚清掉的路径又装回来
            _asyncJob?.Cancel();
            _asyncJob = null;

            _corners.Clear();
            _cornerIndex = 0;
            _stuckTimer = 0f;
            _nextRepathTime = 0f;
            _hasCompletePath = false;
            _hasPendingDestination = false;
            Velocity = Vector2.zero;
            PathEndPosition = Position;
        }

        /// <summary>外部直接改写速度（冲锋刹车、转向等旧逻辑沿用）。</summary>
        public void SetVelocity(Vector2 velocity)
        {
            Velocity = velocity;
        }

        /// <summary>只平移位置、不动路径（局部避障的硬性分离用）。</summary>
        public void Nudge(Vector2 delta)
        {
            if (_transform == null)
            {
                return;
            }

            Vector3 position = _transform.position;
            _transform.position = new Vector3(position.x + delta.x, position.y + delta.y, position.z);
        }

        /// <summary>
        /// 去掉朝某个方向的速度分量（<paramref name="outward"/> 是从对方指向自己的外法线）。
        /// 用于撞在一起时贴着对方滑开，而不是一直顶着。
        /// </summary>
        public void RemoveVelocityAlong(Vector2 outward)
        {
            float inward = Vector2.Dot(Velocity, outward);
            if (inward < 0f)
            {
                Velocity -= outward * inward;
            }
        }

        /// <summary>每帧推进（由宿主调用）。</summary>
        public void Tick(float dt)
        {
            // 掉帧时限幅，避免一帧位移过大直接穿过别人
            dt = Mathf.Min(dt, 0.05f);

            // 后台寻路的结果在这里落地：走到一半停下等路径的单位下一帧就能继续走
            ApplyFinishedPathJob();

            if (!Enabled || IsStopped || dt <= 0f)
            {
                Velocity = Vector2.zero;
                return;
            }

            // 上一帧被预算挡下的目标在这里补算
            if (_hasPendingDestination)
            {
                _hasPendingDestination = false;
                SetDestinationWithStatus(_pendingTarget, _pendingLayer);
            }

            // 静止计时：供上层判断"这算不算一堵墙"
            if (!HasPath || Speed <= 0.01f)
            {
                StaticTime += dt;
            }
            else
            {
                StaticTime = 0f;
            }

            if (!HasPath)
            {
                Velocity = Vector2.zero;

                // 站住的单位也要参与分离，否则会被别人直接穿过
                if (AvoidanceEnabled)
                {
                    AvoidanceSolver.ResolveOverlap(_system, this, dt);
                }

                return;
            }

            Vector2 position = Position;
            Vector2 final = _corners[_corners.Count - 1].Position;

            if (_cornerIndex >= _corners.Count - 1 && (final - position).sqrMagnitude <= _finalReachSqr)
            {
                ResetPath();
                return;
            }

            // 推进已走过的路点
            while (_cornerIndex < _corners.Count - 1 &&
                   (_corners[_cornerIndex].Position - position).sqrMagnitude <= _waypointReachSqr)
            {
                // 跨层：到达门户起点就切到目标层，后续寻路从这里出发
                Layer = _corners[_cornerIndex + 1].Layer;
                _cornerIndex++;
            }

            Vector2 steer = _corners[_cornerIndex].Position;
            Vector2 toSteer = steer - position;
            float distance = toSteer.magnitude;
            if (distance < 0.0001f)
            {
                Velocity = Vector2.zero;
                return;
            }

            Vector2 desiredDir = toSteer / distance;
            float speed = Mathf.Max(0f, Speed);

            // 接近终点时线性自动刹车
            if (_cornerIndex >= _corners.Count - 1 && distance < 1f)
            {
                speed = Mathf.Min(speed, Mathf.Max(0.15f, speed * distance));
            }

            Vector2 desiredVelocity = desiredDir * speed;
            Vector2 nextVelocity = Vector2.MoveTowards(Velocity, desiredVelocity,
                Mathf.Max(0.01f, Acceleration) * dt);

            if (AvoidanceEnabled && nextVelocity.sqrMagnitude > 0.0001f)
            {
                nextVelocity = AvoidanceSolver.Solve(_system, this, nextVelocity);

                // 方向平滑：避障结果逐帧翻转时，直接套用会表现为原地抖动
                nextVelocity = Vector2.Lerp(Velocity, nextVelocity, 1f - Mathf.Exp(-14f * dt));
            }

            Velocity = nextVelocity;

            Vector2 nextPosition = position + Velocity * dt;
            if (_transform != null)
            {
                _transform.position = new Vector3(nextPosition.x, nextPosition.y, _transform.position.z);
            }

            // 已经贴到别人身上的部分用位置修正推开
            if (AvoidanceEnabled)
            {
                AvoidanceSolver.ResolveOverlap(_system, this, dt);
            }

            if (Velocity.sqrMagnitude < 0.0025f)
            {
                _stuckTimer += dt;
            }
            else
            {
                _stuckTimer = 0f;
            }
        }
    }
}
