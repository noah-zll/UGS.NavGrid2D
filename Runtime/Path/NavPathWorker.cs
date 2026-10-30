using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 一次后台寻路请求。提交后请求字段只读；结果字段由工作线程写、主线程在 <see cref="IsDone"/> 之后读。
    /// </summary>
    public sealed class NavPathJob
    {
        // 请求（提交后不再修改）
        internal NavGrid Grid;
        internal INavPortalQuery Portals;
        internal int StartLayer;
        internal int GoalLayer;
        internal Vector2 Start;
        internal Vector2 Goal;
        internal int AreaMask;
        internal float Radius;
        internal bool IgnoreCrowd;
        internal int MaxExpansions;
        internal float MaxMilliseconds;
        internal float HeuristicWeight;

        // 失败后的重试管线（与主线程同步版保持一致）
        internal bool RelaxCrowdOnFail;
        internal bool GreedyFallback;
        internal int GreedyExpansions;
        internal float GreedyMilliseconds;
        internal float GreedyWeight;
        internal int MaxStitchSegments;
        internal float FinalReachSqr;

        /// <summary>结果路径点（不含起点，含终点）。</summary>
        public List<NavPathPoint> Corners { get; } = new List<NavPathPoint>(64);

        public NavPathStatus Status { get; private set; } = NavPathStatus.Invalid;
        public int Expansions { get; private set; }
        public float Milliseconds { get; private set; }
        public bool ReachedGoal { get; private set; }
        public bool Truncated { get; private set; }

        private int _state;

        /// <summary>工作线程已经算完，结果可读。</summary>
        public bool IsDone => Volatile.Read(ref _state) == 1;

        /// <summary>已被主线程取消（改了目标/清空路径），工作线程会尽快退出。</summary>
        public bool IsCancelled => Volatile.Read(ref _state) == -1;

        /// <summary>取消这个请求：已经算完的请求不会被打断。</summary>
        public void Cancel()
        {
            Interlocked.CompareExchange(ref _state, -1, 0);
        }

        internal void Complete(NavPathStatus status, int expansions, float milliseconds, bool reachedGoal, bool truncated)
        {
            Status = status;
            Expansions = expansions;
            Milliseconds = milliseconds;
            ReachedGoal = reachedGoal;
            Truncated = truncated;
            Volatile.Write(ref _state, 1);
        }
    }

    /// <summary>
    /// 寻路工作线程池。
    /// </summary>
    /// <remarks>
    /// 后台线程只读网格数据，所以「重建网格」必须等所有在飞任务结束
    /// （见 <see cref="InFlight"/>，宿主 <see cref="NavGridSystem"/> 据此延后重建）。
    /// 静态/人群数据（<c>StaticBlock</c>）每帧由主线程改写，这里不加锁：
    /// 读到旧值最多让路径多绕开/少绕开一个人，不会造成穿墙，属于可接受的弱一致。
    /// </remarks>
    internal sealed class NavPathWorker
    {
        private readonly ConcurrentQueue<NavPathJob> _pending = new ConcurrentQueue<NavPathJob>();
        private readonly AutoResetEvent _signal = new AutoResetEvent(false);
        private readonly Thread[] _threads;
        private readonly ThreadLocal<WorkerState> _state;

        private int _inFlight;
        private volatile bool _shutdown;

        public NavPathWorker(int threadCount)
        {
            threadCount = Mathf.Clamp(threadCount, 1, 8);
            _threads = new Thread[threadCount];
            _state = new ThreadLocal<WorkerState>(() => new WorkerState());

            for (int i = 0; i < threadCount; i++)
            {
                _threads[i] = new Thread(Run)
                {
                    IsBackground = true,
                    Name = "NavGridPathWorker" + i,
                };
                _threads[i].Start();
            }
        }

        /// <summary>已提交但还没算完的任务数。大于 0 时不能重建网格。</summary>
        public int InFlight => Volatile.Read(ref _inFlight);

        public void Submit(NavPathJob job)
        {
            Interlocked.Increment(ref _inFlight);
            _pending.Enqueue(job);
            _signal.Set();
        }

        public void Shutdown()
        {
            _shutdown = true;
            _signal.Set();
            for (int i = 0; i < _threads.Length; i++)
            {
                NavPathJob job;
                while (_pending.TryDequeue(out job))
                {
                    Interlocked.Decrement(ref _inFlight);
                    job.Cancel();
                }
            }
        }

        private void Run()
        {
            WorkerState state = _state.Value;

            while (!_shutdown)
            {
                NavPathJob job;
                if (_pending.TryDequeue(out job))
                {
                    if (job.IsCancelled)
                    {
                        // 取消的任务也要落一个终态，避免还在等它的代理永远等下去
                        job.Complete(NavPathStatus.Invalid, 0, 0f, false, false);
                    }
                    else
                    {
                        try
                        {
                            Execute(job, state);
                        }
                        catch (System.Exception exception)
                        {
                            // 线程里抛异常会让 _inFlight 永远不归零（网格再也重建不了），这里兜住
                            Debug.LogException(exception);
                            job.Complete(NavPathStatus.Invalid, 0, 0f, false, false);
                        }
                    }

                    Interlocked.Decrement(ref _inFlight);
                    continue;
                }

                _signal.WaitOne(20);
            }
        }

        private static void Execute(NavPathJob job, WorkerState state)
        {
            NavPathfinder finder = state.Finder;
            List<NavPathPoint> buffer = state.Buffer;
            buffer.Clear();

            NavPathStatus status = finder.Search(job.Grid, job.Portals, job.StartLayer, job.Start, job.GoalLayer,
                job.Goal, job.AreaMask, job.Radius, buffer, job.IgnoreCrowd, job.MaxExpansions, job.MaxMilliseconds,
                job.HeuristicWeight, job);

            int expansions = finder.Expansions;
            float milliseconds = finder.Milliseconds;
            bool reached = finder.ReachedGoal;
            bool truncated = finder.Truncated;

            // 目标被人群围住时正常寻路会"到不了"：退一步忽略人群再算一次
            if (status != NavPathStatus.Complete && job.RelaxCrowdOnFail && !job.IgnoreCrowd && !job.IsCancelled)
            {
                List<NavPathPoint> relaxed = state.Relaxed;
                relaxed.Clear();
                NavPathStatus relaxedStatus = finder.Search(job.Grid, job.Portals, job.StartLayer, job.Start,
                    job.GoalLayer, job.Goal, job.AreaMask, job.Radius, relaxed, true, job.MaxExpansions,
                    job.MaxMilliseconds, job.HeuristicWeight, job);

                if (relaxedStatus == NavPathStatus.Complete)
                {
                    buffer.Clear();
                    buffer.AddRange(relaxed);
                    status = relaxedStatus;
                    reached = true;
                    truncated = false;
                    expansions += finder.Expansions;
                    milliseconds += finder.Milliseconds;
                }
            }

            // 预算内没搜完（长绕行）：换强启发再搜一次，它会贴着障碍朝目标推进
            if (status != NavPathStatus.Complete && truncated && job.GreedyFallback && !job.IsCancelled)
            {
                List<NavPathPoint> greedy = state.Greedy;
                greedy.Clear();
                NavPathStatus greedyStatus = finder.Search(job.Grid, job.Portals, job.StartLayer, job.Start,
                    job.GoalLayer, job.Goal, job.AreaMask, job.Radius, greedy, true, job.GreedyExpansions,
                    job.GreedyMilliseconds, job.GreedyWeight, job);

                if (greedyStatus != NavPathStatus.Invalid)
                {
                    buffer.Clear();
                    buffer.AddRange(greedy);
                    status = greedyStatus;
                    reached = finder.ReachedGoal;
                    truncated = finder.Truncated;
                    expansions += finder.Expansions;
                    milliseconds += finder.Milliseconds;
                }
            }

            // 仍被截断：从已算出的终点继续补段
            for (int i = 0; i < job.MaxStitchSegments && status == NavPathStatus.Partial && truncated; i++)
            {
                if (buffer.Count == 0 || job.IsCancelled)
                {
                    break;
                }

                NavPathPoint tail = buffer[buffer.Count - 1];
                if ((tail.Position - job.Goal).sqrMagnitude <= job.FinalReachSqr)
                {
                    break;
                }

                List<NavPathPoint> stitch = state.Stitch;
                stitch.Clear();
                NavPathStatus next = finder.Search(job.Grid, job.Portals, tail.Layer, tail.Position, job.GoalLayer,
                    job.Goal, job.AreaMask, job.Radius, stitch, job.IgnoreCrowd, job.MaxExpansions,
                    job.MaxMilliseconds, job.HeuristicWeight, job);

                if (stitch.Count == 0)
                {
                    break;
                }

                buffer.AddRange(stitch);
                status = next;
                reached = finder.ReachedGoal;
                truncated = finder.Truncated;
                expansions += finder.Expansions;
                milliseconds += finder.Milliseconds;
            }

            job.Corners.Clear();
            job.Corners.AddRange(buffer);
            job.Complete(status, expansions, milliseconds, reached, truncated);
        }

        /// <summary>每个工作线程一份 scratch：A* 的数组、平滑缓冲、输出列表。</summary>
        private sealed class WorkerState
        {
            public readonly NavPathfinder Finder = new NavPathfinder();
            public readonly List<NavPathPoint> Buffer = new List<NavPathPoint>(64);
            public readonly List<NavPathPoint> Relaxed = new List<NavPathPoint>(64);
            public readonly List<NavPathPoint> Greedy = new List<NavPathPoint>(64);
            public readonly List<NavPathPoint> Stitch = new List<NavPathPoint>(64);
        }
    }
}
