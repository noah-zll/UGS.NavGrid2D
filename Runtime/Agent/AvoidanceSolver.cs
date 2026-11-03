using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 局部避障：16 候选方向采样 + 线段最近距离惩罚 + 优先级让行 + 硬性分离。
    /// </summary>
    /// <remarks>
    /// 分工：<see cref="Solve"/> 只负责"绕"，<see cref="ResolveOverlap"/> 负责"不叠"。
    /// 不能靠 Solve 硬性淘汰方向来防穿透 —— 低速时前视距离太短，会把所有方向都判死，
    /// 结果就是互相顶住（挤），所以防穿透统一交给分离 + 速度投影。
    /// </remarks>
    public static class AvoidanceSolver
    {
        private const int DirectionCount = 16;

        /// <summary>预测时长（秒）：用来预测邻居的移动。</summary>
        private const float PredictionTime = 0.5f;

        /// <summary>方向采样的最小前视距离（世界单位）：低速时也能提前绕开。</summary>
        private const float MinLookAhead = 1.2f;

        /// <summary>邻居查询余量（世界单位）。</summary>
        private const float NeighborMargin = 2.5f;

        /// <summary>
        /// 方向粘性：给"和当前速度同向"的候选加分。
        /// 拥挤时左右两个对称方向的得分几乎一样，没有这一项就会逐帧翻边，看起来是来回抖。
        /// 权重明显小于 <c>Dot(dir, desiredDir)</c>，所以真需要转向时照样转得过去。
        /// </summary>
        private const float DirectionStickiness = 0.5f;

        private static readonly List<GridAgent> Neighbors = new List<GridAgent>(32);

        /// <summary>在给定期望速度的基础上选一个会绕开邻居的方向。</summary>
        public static Vector2 Solve(NavGridSystem system, GridAgent agent, Vector2 desiredVelocity)
        {
            float speed = desiredVelocity.magnitude;
            if (system == null || speed < 0.01f)
            {
                return desiredVelocity;
            }

            system.QueryNeighbors(agent, agent.Radius + NeighborMargin, Neighbors);
            if (Neighbors.Count == 0)
            {
                return desiredVelocity;
            }

            NavGridSettings settings = system.Settings;
            Vector2 position = agent.Position;
            Vector2 desiredDir = desiredVelocity / speed;
            float lookAhead = Mathf.Max(speed * PredictionTime, MinLookAhead);
            float selfRadius = agent.Radius;

            // 当前实际朝向：用来给候选方向加"别突然换边"的粘性分
            Vector2 currentDir = agent.Velocity.sqrMagnitude > 0.0001f
                ? agent.Velocity.normalized
                : desiredDir;

            float bestScore = float.MinValue;
            Vector2 bestDir = desiredDir;
            bool found = false;

            for (int i = 0; i < DirectionCount; i++)
            {
                Vector2 dir = Rotate(desiredDir, i * (360f / DirectionCount));
                Vector2 predicted = position + dir * lookAhead;

                // 静态障碍（墙/建筑）直接淘汰该方向
                if (!system.IsPassable(predicted, agent.AreaMask, agent.Radius))
                {
                    continue;
                }

                float score = Vector2.Dot(dir, desiredDir) + DirectionStickiness * Vector2.Dot(dir, currentDir);

                for (int n = 0; n < Neighbors.Count; n++)
                {
                    GridAgent other = Neighbors[n];
                    float minDistance = selfRadius + other.Radius;
                    if (minDistance <= 0.0001f)
                    {
                        continue;
                    }

                    Vector2 otherPredicted = other.Position + other.Velocity * PredictionTime;

                    // 用整条前视线段到邻居的最近距离，而不是只看端点
                    float distance = DistancePointToSegment(otherPredicted, position, predicted);

                    if (distance < minDistance)
                    {
                        score -= 1.2f * (1f - distance / minDistance);
                    }
                    else if (distance < minDistance * 1.6f)
                    {
                        score -= 0.25f * (1f - distance / (minDistance * 1.6f));
                    }

                    // 优先级数值越小优先级越高：给高优先级让路
                    if (other.AvoidancePriority < agent.AvoidancePriority)
                    {
                        Vector2 toOther = other.Position - position;
                        if (toOther.sqrMagnitude > 0.0001f && Vector2.Dot(dir, toOther.normalized) > 0.3f)
                        {
                            score -= 0.3f;
                        }
                    }
                }

                if (!found || score > bestScore)
                {
                    found = true;
                    bestScore = score;
                    bestDir = dir;
                }
            }

            if (!found)
            {
                return desiredVelocity;
            }

            // 最优方向也偏得厉害：被附近的单位堵住了。
            // 这里既不能全体停下（互相等着 = 死锁，谁都不动），
            // 也不能全体都蹭（16 方向里选出来的最优方向会左右翻转，表现为来回挤）。
            // 改成按「避障优先级 → Id」让行：同一堆里只有 Id 最小的那个往前推，其余原地等。
            // 它走开之后下一个自然解除封锁，所以不会死锁，也不会来回换方向。
            if (bestScore < settings.StopScoreThreshold)
            {
                if (HasHigherPriorityBlocker(agent, desiredDir, position))
                {
                    return Vector2.zero;
                }

                return desiredDir * (speed * 0.35f);
            }

            // 就算一直在赔分也保留一点速度，避免原地僵死
            return bestDir * (speed * Mathf.Clamp(bestScore, 0.3f, 1f));
        }

        /// <summary>
        /// 硬性分离：已经和人叠在一起时按重叠量把位置推开，并消掉朝对方的速度分量。
        /// </summary>
        public static void ResolveOverlap(NavGridSystem system, GridAgent agent, float dt)
        {
            if (system == null)
            {
                return;
            }

            system.QueryNeighbors(agent, agent.Radius * 2f + 0.5f, Neighbors);
            if (Neighbors.Count == 0)
            {
                return;
            }

            float deadZone = system.Settings.OverlapDeadZone;
            Vector2 position = agent.Position;
            Vector2 correction = Vector2.zero;

            for (int i = 0; i < Neighbors.Count; i++)
            {
                GridAgent other = Neighbors[i];
                float minDistance = agent.Radius + other.Radius;
                if (minDistance <= 0.0001f)
                {
                    continue;
                }

                Vector2 delta = position - other.Position;
                float distance = delta.magnitude;
                float overlap = minDistance - distance;

                // 留一点死区：轻微贴合不纠正，否则一大群单位挤在一起会持续互相推、看起来在抖
                if (overlap < deadZone)
                {
                    continue;
                }

                Vector2 direction;
                if (distance > 0.0001f)
                {
                    direction = delta / distance;
                }
                else
                {
                    // 完全重合：用 Id 生成一个确定性的分离方向
                    float angle = agent.Id * 1.7f;
                    direction = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
                }

                // 自己在移动时：对方也动各让一半，对方站着则自己全让；
                // 自己站着（防守/阵型）时几乎不让位，避免被别人硬挤出一个缺口。
                float weight = agent.HasPath ? (other.HasPath ? 0.5f : 1f) : 0.15f;
                correction += direction * (overlap * weight);

                // 消掉朝对方的速度分量，避免"顶牛"
                agent.RemoveVelocityAlong(direction);
            }

            if (correction.sqrMagnitude <= 0.000001f)
            {
                return;
            }

            float maxStep = Mathf.Max(0.25f, agent.Speed) * dt + 0.02f;
            if (correction.magnitude > maxStep)
            {
                correction = correction.normalized * maxStep;
            }

            if (system.IsPassable(position + correction, agent.AreaMask, agent.Radius))
            {
                agent.Nudge(correction);
                return;
            }

            // 靠墙时退化成单轴修正
            Vector2 xOnly = new Vector2(correction.x, 0f);
            if (Mathf.Abs(correction.x) > 0.0001f &&
                system.IsPassable(position + xOnly, agent.AreaMask, agent.Radius))
            {
                agent.Nudge(xOnly);
                return;
            }

            Vector2 yOnly = new Vector2(0f, correction.y);
            if (Mathf.Abs(correction.y) > 0.0001f &&
                system.IsPassable(position + yOnly, agent.AreaMask, agent.Radius))
            {
                agent.Nudge(yOnly);
            }
        }

        /// <summary>
        /// 前进方向上是否有比自己更优先的单位（数值小的优先级更高，同级按 Id 小的优先）。
        /// 有就让行，没有就自己往前推：一堆单位里永远只有 Id 最小的那个在动，不会互相顶。
        /// </summary>
        private static bool HasHigherPriorityBlocker(GridAgent agent, Vector2 desiredDir, Vector2 position)
        {
            for (int i = 0; i < Neighbors.Count; i++)
            {
                GridAgent other = Neighbors[i];

                Vector2 toOther = other.Position - position;
                float forward = Vector2.Dot(toOther, desiredDir);
                if (forward <= 0f)
                {
                    // 只让"挡在前进方向上"的人，身后的不管
                    continue;
                }

                float reach = agent.Radius + other.Radius + 1f;
                if (toOther.sqrMagnitude > reach * reach)
                {
                    continue;
                }

                if (other.AvoidancePriority < agent.AvoidancePriority ||
                    (other.AvoidancePriority == agent.AvoidancePriority && other.Id < agent.Id))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>点到线段的最近距离。</summary>
        private static float DistancePointToSegment(Vector2 point, Vector2 from, Vector2 to)
        {
            Vector2 segment = to - from;
            float lengthSqr = segment.sqrMagnitude;
            if (lengthSqr <= 0.000001f)
            {
                return Vector2.Distance(point, from);
            }

            float t = Mathf.Clamp01(Vector2.Dot(point - from, segment) / lengthSqr);
            return Vector2.Distance(point, from + segment * t);
        }

        private static Vector2 Rotate(Vector2 vector, float degrees)
        {
            float radians = degrees * Mathf.Deg2Rad;
            float cos = Mathf.Cos(radians);
            float sin = Mathf.Sin(radians);
            return new Vector2(vector.x * cos - vector.y * sin, vector.x * sin + vector.y * cos);
        }
    }
}
