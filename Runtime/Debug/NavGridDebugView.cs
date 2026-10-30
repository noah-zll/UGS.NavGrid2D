using System.Collections.Generic;
using UnityEngine;

namespace UGS.NavGrid2D
{
    /// <summary>
    /// 网格寻路调试视图。
    /// <list type="bullet">
    /// <item>右上角小地图：不可通行格、窄格、人群密度热力；</item>
    /// <item>游戏画面：每个网格代理的"障碍范围"（分离半径），重叠时变红。</item>
    /// </list>
    /// </summary>
    /// <remarks>
    /// 通过 <see cref="EnsureCreated"/> 自动创建；<see cref="visible"/> 控制显隐。
    /// 只依赖公开 API，业务侧也可以自己挂组件并按需设置 <see cref="system"/>。
    /// </remarks>
    public sealed class NavGridDebugView : MonoBehaviour
    {
        [Tooltip("要显示的网格系统；为空时使用第一个自动创建的系统")]
        public NavGridSystem system;

        [Tooltip("小地图边长（像素）")]
        public int textureSize = 400;

        [Tooltip("是否启用")]
        public bool visible = true;

        [Tooltip("在游戏画面里画出每个代理的障碍范围")]
        public bool showAgentRanges = true;

        [Tooltip("小地图上叠加人群密度热力")]
        public bool showDensity = true;

        [Tooltip("显示哪一层（0 = 地面）")]
        public int layer;

        private const int DensityRefreshFrames = 10;

        private static readonly Color OverlapColor = new Color(1f, 0.25f, 0.2f, 0.9f);
        private static readonly Color MovingColor = new Color(0.2f, 1f, 0.45f, 0.8f);
        private static readonly Color BlockedColor = new Color(1f, 0.75f, 0.2f, 0.85f);
        private static readonly Color IdleColor = new Color(1f, 1f, 1f, 0.45f);
        private static readonly Color DisabledColor = new Color(0.45f, 0.45f, 0.45f, 0.3f);

        private Texture2D _texture;
        private Texture2D _ringTexture;
        private int _builtVersion = -1;
        private int _builtDensityFrame = -100;

        private readonly List<GridAgent> _agents = new List<GridAgent>(512);

        private static NavGridDebugView _instance;

        /// <summary>创建（或复用）调试视图并绑定到指定网格系统。</summary>
        public static NavGridDebugView EnsureCreated(NavGridSystem target = null)
        {
            if (_instance == null)
            {
                GameObject viewObject = new GameObject("NavGridDebugView");
                Object.DontDestroyOnLoad(viewObject);
                _instance = viewObject.AddComponent<NavGridDebugView>();
            }

            if (target != null)
            {
                _instance.system = target;
            }

            return _instance;
        }

        private void OnDestroy()
        {
            if (_instance == this)
            {
                _instance = null;
            }
        }

        private void OnGUI()
        {
            if (!visible || system == null)
            {
                return;
            }

            NavGrid grid = system.Grid;
            if (grid == null)
            {
                return;
            }

            system.CopyAgents(_agents);

            if (showAgentRanges)
            {
                DrawAgentRanges();
            }

            DrawMiniMap(grid);
        }

        #region 代理障碍范围

        private void DrawAgentRanges()
        {
            Camera camera = Camera.main;
            if (camera == null)
            {
                return;
            }

            EnsureRingTexture();

            float screenHeight = Screen.height;

            for (int i = 0; i < _agents.Count; i++)
            {
                GridAgent agent = _agents[i];
                if (agent == null)
                {
                    continue;
                }

                Vector2 worldPosition = agent.Position;
                Vector3 world = new Vector3(worldPosition.x, worldPosition.y, 0f);
                Vector3 screen = camera.WorldToScreenPoint(world);
                if (screen.z <= 0f)
                {
                    continue;
                }

                Vector3 screenRight = camera.WorldToScreenPoint(world + Vector3.right * agent.Radius);
                float screenRadius = Mathf.Abs(screenRight.x - screen.x);
                if (screenRadius < 1f)
                {
                    continue;
                }

                GUI.color = agent.Enabled ? GetRangeColor(agent, i) : DisabledColor;

                float size = screenRadius * 2f;
                GUI.DrawTexture(new Rect(screen.x - screenRadius, screenHeight - screen.y - screenRadius, size, size),
                    _ringTexture);
            }

            GUI.color = Color.white;
        }

        /// <summary>重叠判定容差：只有明显插进去才算重叠，贴在一起不算。</summary>
        private const float OverlapTolerance = 0.3f;

        /// <summary>真正重叠标红；想走但走不动（被堵）标橙；在移动标绿；其余白。</summary>
        private Color GetRangeColor(GridAgent agent, int index)
        {
            if (_agents.Count <= 800)
            {
                Vector2 position = agent.Position;
                for (int j = 0; j < _agents.Count; j++)
                {
                    if (j == index)
                    {
                        continue;
                    }

                    GridAgent other = _agents[j];
                    if (other == null || !other.Enabled)
                    {
                        continue;
                    }

                    float overlapDistance = agent.Radius + other.Radius - OverlapTolerance;
                    if ((other.Position - position).sqrMagnitude < overlapDistance * overlapDistance)
                    {
                        return OverlapColor;
                    }
                }
            }

            if (agent.Velocity.sqrMagnitude > 0.01f)
            {
                return MovingColor;
            }

            // 有路径、也想走，但实际没动 = 被挡住/在等位置
            return agent.HasPath && agent.Speed > 0.01f ? BlockedColor : IdleColor;
        }

        private void EnsureRingTexture()
        {
            if (_ringTexture != null)
            {
                return;
            }

            const int size = 64;
            const float outer = size * 0.5f;
            const float thickness = 2.5f;

            _ringTexture = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Bilinear,
                wrapMode = TextureWrapMode.Clamp,
            };

            Color32[] pixels = new Color32[size * size];
            Color32 clear = new Color32(255, 255, 255, 0);
            Color32 solid = new Color32(255, 255, 255, 255);

            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    float dx = x + 0.5f - outer;
                    float dy = y + 0.5f - outer;
                    float distance = Mathf.Sqrt(dx * dx + dy * dy);
                    pixels[y * size + x] = Mathf.Abs(distance - (outer - thickness)) <= thickness ? solid : clear;
                }
            }

            _ringTexture.SetPixels32(pixels);
            _ringTexture.Apply(false);
        }

        #endregion

        #region 小地图

        private void DrawMiniMap(NavGrid grid)
        {
            EnsureTexture();

            bool densityRefresh = showDensity && Time.frameCount - _builtDensityFrame >= DensityRefreshFrames;
            if (_builtVersion != grid.Version || densityRefresh)
            {
                RebuildTexture(grid);
                _builtVersion = grid.Version;
                _builtDensityFrame = Time.frameCount;
            }

            float x = Screen.width - textureSize - 16f;
            GUI.Box(new Rect(x - 8f, 8f, textureSize + 16f, textureSize + 40f), GUIContent.none);
            GUI.Label(new Rect(x, 12f, textureSize, 20f),
                $"NavGrid v{grid.Version}  regions={grid.RegionCount}  agents={system.AgentCount}");
            GUI.DrawTexture(new Rect(x, 32f, textureSize, textureSize), _texture);
        }

        private void EnsureTexture()
        {
            int size = Mathf.Max(32, textureSize);
            if (_texture != null && _texture.width == size)
            {
                return;
            }

            _texture = new Texture2D(size, size, TextureFormat.RGBA32, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
            };
            _builtVersion = -1;
            _builtDensityFrame = -100;
        }

        private void RebuildTexture(NavGrid grid)
        {
            int size = _texture.width;
            Color32[] pixels = new Color32[size * size];

            Color32 blocked = new Color32(20, 20, 20, 255);
            Color32 open = new Color32(190, 200, 190, 255);
            Color32 narrow = new Color32(230, 170, 90, 255);
            Color32 crowded = new Color32(255, 60, 50, 255);

            NavGridSettings settings = grid.Settings;
            int cellsPerPixel = Mathf.Max(1, grid.Side / size);
            int obstacleSide = settings.ObstacleSide;
            int cellsPerObstacleCell = Mathf.Max(1, settings.CellsPerObstacleCell);
            byte regionClearanceNeed = settings.RegionClearanceNeed;

            int layerIndex = Mathf.Clamp(layer, 0, grid.LayerCount - 1);
            int layerOffset = layerIndex * grid.LayerCellCount;

            for (int py = 0; py < size; py++)
            {
                int cellY = py * cellsPerPixel;
                int rowBase = py * size;

                for (int px = 0; px < size; px++)
                {
                    int cellX = px * cellsPerPixel;
                    int index = layerOffset + grid.ToIndex(
                        Mathf.Clamp(cellX, 0, grid.Side - 1),
                        Mathf.Clamp(cellY, 0, grid.Side - 1));

                    if (grid.Area[index] == NavGrid.Impassable)
                    {
                        pixels[rowBase + px] = blocked;
                        continue;
                    }

                    Color32 color = grid.Clearance[index] < regionClearanceNeed ? narrow : open;

                    if (showDensity)
                    {
                        int densityX = Mathf.Clamp(cellX / cellsPerObstacleCell, 0, obstacleSide - 1);
                        int densityY = Mathf.Clamp(cellY / cellsPerObstacleCell, 0, obstacleSide - 1);
                        int density = grid.Density[densityX * obstacleSide + densityY];

                        if (density > 0)
                        {
                            color = Color32.Lerp(color, crowded, Mathf.Clamp01(density / 8f));
                        }
                    }

                    pixels[rowBase + px] = color;
                }
            }

            _texture.SetPixels32(pixels);
            _texture.Apply(false);
        }

        #endregion
    }
}
