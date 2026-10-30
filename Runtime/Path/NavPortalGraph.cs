using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 门户：连接两个格子（可跨层）。层之间只能通过门户连通。
    /// </summary>
    /// <remarks>
    /// <see cref="CellA"/> / <see cref="CellB"/> 都是**全局格索引**（层号 × 单层格数 + 层内索引），
    /// 可用 <see cref="NavGrid.GlobalIndex"/> 构造。世界坐标由 <see cref="NavGrid.CellCenter(int)"/> 还原。
    /// </remarks>
    public struct NavPortal
    {
        /// <summary>
        /// 业务自定义的类型标签，框架不做任何解释（只用于业务侧分类、表现与调试）。
        /// 业务侧自己定义枚举，赋值时强转成 byte。
        /// </summary>
        public byte Kind;
        public int CellA;
        public int CellB;

        /// <summary>false 时只能 A → B。</summary>
        public bool Bidirectional;

        /// <summary>城门开闭、梯子被摧毁等。</summary>
        public bool Enabled;

        /// <summary>穿越时上层可用于表现（等价于 isOnOffMeshLink）。</summary>
        public bool IsOffMeshLike;

        /// <summary>穿越速度（世界单位/秒），参与 A* 的跨层代价。</summary>
        public float TraverseSpeed;
    }

    /// <summary>
    /// 只读门户查询。后台线程用 <see cref="NavPortalSnapshot"/>（不可变），
    /// 主线程同步寻路直接用 <see cref="NavPortalGraph"/>。
    /// </summary>
    public interface INavPortalQuery
    {
        int Count { get; }
        IReadOnlyList<int> Query(int globalCell);
        NavPortal this[int index] { get; }
        bool TryTraverse(NavPortal portal, int from, out int to);
    }

    /// <summary>
    /// 门户图的不可变快照：创建后不再改动，可以安全地被多个后台线程同时查询。
    /// </summary>
    /// <remarks>
    /// 直接用 <see cref="NavPortalGraph"/> 会踩两个并发问题：它的 <c>Query</c> 复用了内部缓冲，
    /// 邻接表也还是可变的 <see cref="Dictionary{TKey,TValue}"/>——主线程加门户时后台正在查就会炸。
    /// 这里把门户数组与邻接表都拷成只读结构，<see cref="Query"/> 直接返回数组本身，零分配。
    /// </remarks>
    public sealed class NavPortalSnapshot : INavPortalQuery
    {
        private static readonly int[] Empty = new int[0];

        private readonly NavPortal[] _portals;
        private readonly Dictionary<int, int[]> _cells;

        public NavPortalSnapshot(NavPortalGraph source)
        {
            if (source == null)
            {
                _portals = new NavPortal[0];
                _cells = new Dictionary<int, int[]>(0);
                return;
            }

            IReadOnlyList<NavPortal> items = source.Items;
            _portals = new NavPortal[items.Count];
            for (int i = 0; i < _portals.Length; i++)
            {
                _portals[i] = items[i];
            }

            Dictionary<int, List<int>> cells = source.Cells;
            _cells = new Dictionary<int, int[]>(cells.Count);
            foreach (KeyValuePair<int, List<int>> pair in cells)
            {
                _cells[pair.Key] = pair.Value.Count == 0 ? Empty : pair.Value.ToArray();
            }
        }

        public int Count => _portals.Length;

        public NavPortal this[int index] => _portals[index];

        public IReadOnlyList<int> Query(int globalCell)
        {
            return _cells.TryGetValue(globalCell, out int[] list) ? list : Empty;
        }

        public bool TryTraverse(NavPortal portal, int from, out int to)
        {
            if (portal.Enabled)
            {
                if (portal.CellA == from)
                {
                    to = portal.CellB;
                    return true;
                }

                if (portal.Bidirectional && portal.CellB == from)
                {
                    to = portal.CellA;
                    return true;
                }
            }

            to = -1;
            return false;
        }
    }

    /// <summary>
    /// 门户图：维护「格 → 门户」邻接表，增删改不触发栅格重算，只自增版本。
    /// </summary>
    public sealed class NavPortalGraph : INavPortalQuery
    {
        private readonly List<NavPortal> _portals = new List<NavPortal>(64);
        private readonly Dictionary<int, List<int>> _cells = new Dictionary<int, List<int>>(128);
        private readonly List<int> _queryBuffer = new List<int>(8);

        /// <summary>门户列表（构建 <see cref="NavPortalSnapshot"/> 用）。</summary>
        internal IReadOnlyList<NavPortal> Items => _portals;

        /// <summary>邻接表（构建 <see cref="NavPortalSnapshot"/> 用）。</summary>
        internal Dictionary<int, List<int>> Cells => _cells;

        /// <summary>门户图版本，发生变化时自增（供路径缓存失效）。</summary>
        public int Version { get; private set; }

        public int Count => _portals.Count;

        public NavPortal this[int index] => _portals[index];

        public void Clear()
        {
            _portals.Clear();
            _cells.Clear();
            Version++;
        }

        /// <summary>登记一个门户，返回其索引。</summary>
        public int Add(NavPortal portal)
        {
            int index = _portals.Count;
            _portals.Add(portal);

            AddAdjacency(portal.CellA, index);
            if (portal.Bidirectional)
            {
                AddAdjacency(portal.CellB, index);
            }

            Version++;
            return index;
        }

        public void SetEnabled(int index, bool enabled)
        {
            if (index < 0 || index >= _portals.Count)
            {
                return;
            }

            NavPortal portal = _portals[index];
            if (portal.Enabled == enabled)
            {
                return;
            }

            portal.Enabled = enabled;
            _portals[index] = portal;
            Version++;
        }

        public void RemoveAt(int index)
        {
            if (index < 0 || index >= _portals.Count)
            {
                return;
            }

            NavPortal portal = _portals[index];
            RemoveAdjacency(portal.CellA, index);
            if (portal.Bidirectional)
            {
                RemoveAdjacency(portal.CellB, index);
            }

            _portals.RemoveAt(index);

            // 索引整体前移，邻接表需要整体重建
            RebuildAdjacency();
            Version++;
        }

        /// <summary>查询与某格相连的门户索引（结果复用内部缓冲，调用方不要长期持有）。</summary>
        public IReadOnlyList<int> Query(int globalCell)
        {
            _queryBuffer.Clear();
            if (_cells.TryGetValue(globalCell, out List<int> list))
            {
                _queryBuffer.AddRange(list);
            }

            return _queryBuffer;
        }

        /// <summary>从 from 出发能否穿过该门户；能则给出目的地格。</summary>
        public bool TryTraverse(NavPortal portal, int from, out int to)
        {
            if (portal.Enabled)
            {
                if (portal.CellA == from)
                {
                    to = portal.CellB;
                    return true;
                }

                if (portal.Bidirectional && portal.CellB == from)
                {
                    to = portal.CellA;
                    return true;
                }
            }

            to = -1;
            return false;
        }

        private void AddAdjacency(int cell, int index)
        {
            if (!_cells.TryGetValue(cell, out List<int> list))
            {
                list = new List<int>(2);
                _cells[cell] = list;
            }

            list.Add(index);
        }

        private void RemoveAdjacency(int cell, int index)
        {
            if (_cells.TryGetValue(cell, out List<int> list))
            {
                list.Remove(index);
                if (list.Count == 0)
                {
                    _cells.Remove(cell);
                }
            }
        }

        private void RebuildAdjacency()
        {
            _cells.Clear();
            for (int i = 0; i < _portals.Count; i++)
            {
                NavPortal portal = _portals[i];
                AddAdjacency(portal.CellA, i);
                if (portal.Bidirectional)
                {
                    AddAdjacency(portal.CellB, i);
                }
            }
        }
    }
}
