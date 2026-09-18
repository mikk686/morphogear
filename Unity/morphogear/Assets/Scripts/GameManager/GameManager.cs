using System;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;
using UnityEngine.UI;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace RobotRuntime
{
    [DefaultExecutionOrder(-1000)]
    public sealed class GameManager : MonoBehaviour
    {
        public static GameManager Instance { get; private set; }

        [Header("Startup")]
        [SerializeField] private RobotMode startingMode = RobotMode.Game;
        [SerializeField] private bool applyStartingModeOnStart = true;
        [SerializeField] private bool pauseGameWhileMenuIsOpen = true;
        [SerializeField] private bool persistBetweenScenes;

        [Header("Mode Profiles")]
        [SerializeField] private List<ModeProfile> profiles = new List<ModeProfile>();

        [Header("Events")]
        public UnityEvent onBeforeModeChanged;
        public UnityEvent onModeChanged;

        public RobotMode ActiveMode { get; private set; }
        public bool IsMenuOpen => menuRoot != null && menuRoot.activeSelf;
        public event Action<RobotMode> ModeChanged;

        private Canvas canvas;
        private GameObject menuRoot;
        private RectTransform tabRow;
        private RectTransform settingsRoot;
        private Text titleText;
        private Text descriptionText;
        private Text statusText;
        private Button applyButton;
        private readonly List<Button> tabs = new List<Button>();
        private readonly List<GameObject> spawnedModeObjects = new List<GameObject>();
        private readonly Dictionary<MemberOverride, string> pendingValues = new Dictionary<MemberOverride, string>();
        private RobotMode selectedMode;
        private float timeScaleBeforeMenu = 1f;
        private Font builtInFont;

        private static readonly Color Navy = new Color32(12, 17, 27, 247);
        private static readonly Color Panel = new Color32(22, 29, 43, 255);
        private static readonly Color PanelLight = new Color32(31, 40, 57, 255);
        private static readonly Color Muted = new Color32(148, 160, 180, 255);
        private static readonly Color White = new Color32(239, 244, 252, 255);

        private void Reset()
        {
            profiles = new List<ModeProfile>
            {
                NewProfile(RobotMode.Game, "GAME", "Standalone Unity world. No ROS connection.", new Color32(80, 200, 255, 255)),
                NewProfile(RobotMode.Mirror, "MIRROR", "Unity commands are mirrored by the connected robot.", new Color32(170, 105, 255, 255)),
                NewProfile(RobotMode.Simulation, "SIMULATION", "Commands go to the robot; telemetry is supplied by Unity.", new Color32(255, 166, 72, 255)),
                NewProfile(RobotMode.Visualisation, "VISUALISATION", "Robot actions and telemetry drive the Unity representation.", new Color32(83, 224, 156, 255))
            };
        }

        private static ModeProfile NewProfile(RobotMode mode, string title, string description, Color accent)
        {
            return new ModeProfile { mode = mode, title = title, description = description, accent = accent };
        }

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }

            Instance = this;
            if (persistBetweenScenes) DontDestroyOnLoad(gameObject);
            EnsureProfilesExist();
            BuildHud();
            menuRoot.SetActive(false);
        }

        private void Start()
        {
            if (applyStartingModeOnStart) ApplyMode(startingMode);
            else
            {
                ActiveMode = startingMode;
                SelectMode(startingMode);
            }
        }

        private void Update()
        {
            bool escapePressed;
#if ENABLE_INPUT_SYSTEM
            escapePressed = Keyboard.current != null && Keyboard.current.escapeKey.wasPressedThisFrame;
#else
            escapePressed = Input.GetKeyDown(KeyCode.Escape);
#endif
            if (escapePressed) ToggleMenu();
        }

        private void OnDestroy()
        {
            if (Instance == this) Instance = null;
            if (IsMenuOpen && pauseGameWhileMenuIsOpen) Time.timeScale = timeScaleBeforeMenu;
        }

        public void ToggleMenu() => SetMenuOpen(!IsMenuOpen);

        public void SetMenuOpen(bool open)
        {
            if (menuRoot == null || menuRoot.activeSelf == open) return;
            menuRoot.SetActive(open);

            if (pauseGameWhileMenuIsOpen)
            {
                if (open)
                {
                    timeScaleBeforeMenu = Time.timeScale;
                    Time.timeScale = 0f;
                }
                else Time.timeScale = timeScaleBeforeMenu;
            }

            if (open) SelectMode(ActiveMode);
        }

        public void ApplySelectedMode()
        {
            CommitPendingSettings(CurrentProfile());
            ApplyMode(selectedMode);
            SetMenuOpen(false);
        }

        public void ApplyMode(RobotMode mode)
        {
            ModeProfile profile = FindProfile(mode);
            if (profile == null)
            {
                Debug.LogError($"No profile exists for mode {mode}.", this);
                return;
            }

            onBeforeModeChanged?.Invoke();
            DestroySpawnedObjects();
            DisableAllManagedComponents();
            ApplySettings(profile);
            SetProfileComponents(profile);
            SpawnStartupPrefabs(profile);

            ActiveMode = mode;
            selectedMode = mode;
            RefreshStatus();
            onModeChanged?.Invoke();
            ModeChanged?.Invoke(mode);
            Debug.Log($"Robot mode applied: {mode}", this);
        }

        private void DisableAllManagedComponents()
        {
            var visited = new HashSet<Behaviour>();
            foreach (ModeProfile profile in profiles)
                foreach (ComponentToggle toggle in profile.sceneComponents)
                    if (toggle != null && toggle.component != null && visited.Add(toggle.component))
                        toggle.component.enabled = false;
        }

        private void SetProfileComponents(ModeProfile profile)
        {
            foreach (ComponentToggle toggle in profile.sceneComponents)
                if (toggle != null && toggle.component != null)
                    toggle.component.enabled = toggle.enabled;
        }

        private void ApplySettings(ModeProfile profile)
        {
            foreach (MemberOverride setting in profile.settings)
            {
                if (setting == null || setting.target == null || string.IsNullOrWhiteSpace(setting.memberName)) continue;
                object value = GetOverrideValue(setting);
                Type type = setting.target.GetType();
                FieldInfo field = type.GetField(setting.memberName, BindingFlags.Instance | BindingFlags.Public);
                PropertyInfo property = type.GetProperty(setting.memberName, BindingFlags.Instance | BindingFlags.Public);

                try
                {
                    if (field != null) field.SetValue(setting.target, ConvertValue(value, field.FieldType));
                    else if (property != null && property.CanWrite) property.SetValue(setting.target, ConvertValue(value, property.PropertyType));
                    else Debug.LogWarning($"Public field/property '{setting.memberName}' was not found on {type.Name}.", setting.target);
                }
                catch (Exception exception)
                {
                    Debug.LogError($"Could not apply {type.Name}.{setting.memberName}: {exception.Message}", setting.target);
                }
            }
        }

        private static object GetOverrideValue(MemberOverride setting)
        {
            switch (setting.valueType)
            {
                case OverrideValueType.Boolean: return setting.boolValue;
                case OverrideValueType.Integer: return setting.intValue;
                case OverrideValueType.Float: return setting.floatValue;
                case OverrideValueType.String: return setting.stringValue;
                case OverrideValueType.Vector3: return setting.vector3Value;
                case OverrideValueType.Color: return setting.colorValue;
                default: return null;
            }
        }

        private static object ConvertValue(object value, Type destination)
        {
            if (value == null || destination.IsInstanceOfType(value)) return value;
            if (destination.IsEnum) return Enum.Parse(destination, value.ToString(), true);
            return Convert.ChangeType(value, destination, CultureInfo.InvariantCulture);
        }

        private void SpawnStartupPrefabs(ModeProfile profile)
        {
            foreach (GameObject prefab in profile.startupPrefabs)
            {
                if (prefab == null) continue;
                GameObject instance = Instantiate(prefab, transform);
                instance.name = $"[Mode {profile.mode}] {prefab.name}";
                spawnedModeObjects.Add(instance);
                foreach (MonoBehaviour behaviour in instance.GetComponentsInChildren<MonoBehaviour>(true))
                    if (behaviour is IModeInitializable initializable)
                        initializable.InitializeForMode(profile.mode, this);
            }
        }

        private void DestroySpawnedObjects()
        {
            foreach (GameObject item in spawnedModeObjects)
                if (item != null) Destroy(item);
            spawnedModeObjects.Clear();
        }

        private void EnsureProfilesExist()
        {
            foreach (RobotMode mode in Enum.GetValues(typeof(RobotMode)))
                if (FindProfile(mode) == null)
                    profiles.Add(NewProfile(mode, mode.ToString().ToUpperInvariant(), "Configure this mode in the GameManager Inspector.", Color.cyan));
        }

        private ModeProfile FindProfile(RobotMode mode) => profiles.Find(profile => profile != null && profile.mode == mode);
        private ModeProfile CurrentProfile() => FindProfile(selectedMode);

        // ---------------- Runtime-generated uGUI ----------------
        private void BuildHud()
        {
            builtInFont = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            EnsureEventSystem();

            GameObject canvasObject = NewUi("Robot Mode HUD", transform);
            canvas = canvasObject.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = 500;
            canvasObject.AddComponent<GraphicRaycaster>();
            CanvasScaler scaler = canvasObject.AddComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.matchWidthOrHeight = 0.5f;

            menuRoot = NewUi("Menu", canvasObject.transform);
            Stretch(menuRoot.GetComponent<RectTransform>());
            Image dim = menuRoot.AddComponent<Image>();
            dim.color = new Color(0.01f, 0.015f, 0.025f, 0.74f);

            GameObject shell = PanelObject("Shell", menuRoot.transform, Navy);
            RectTransform shellRect = shell.GetComponent<RectTransform>();
            shellRect.anchorMin = new Vector2(0.5f, 0.5f);
            shellRect.anchorMax = new Vector2(0.5f, 0.5f);
            shellRect.pivot = new Vector2(0.5f, 0.5f);
            shellRect.sizeDelta = new Vector2(1120f, 700f);
            Shadow shadow = shell.AddComponent<Shadow>();
            shadow.effectColor = new Color(0, 0, 0, 0.65f);
            shadow.effectDistance = new Vector2(0, -14);

            VerticalLayoutGroup shellLayout = shell.AddComponent<VerticalLayoutGroup>();
            shellLayout.padding = new RectOffset(48, 48, 36, 36);
            shellLayout.spacing = 22;
            shellLayout.childControlHeight = true;
            shellLayout.childForceExpandHeight = false;

            GameObject header = NewUi("Header", shell.transform);
            header.AddComponent<LayoutElement>().preferredHeight = 78;
            HorizontalLayoutGroup headerLayout = header.AddComponent<HorizontalLayoutGroup>();
            headerLayout.childAlignment = TextAnchor.MiddleLeft;
            headerLayout.childForceExpandWidth = false;
            headerLayout.spacing = 20;

            GameObject headerCopy = NewUi("Header Copy", header.transform);
            headerCopy.AddComponent<LayoutElement>().flexibleWidth = 1;
            VerticalLayoutGroup headerCopyLayout = headerCopy.AddComponent<VerticalLayoutGroup>();
            headerCopyLayout.childForceExpandHeight = false;
            AddText(headerCopy.transform, "SYSTEM CONTROL", 14, Muted, FontStyle.Bold).gameObject.AddComponent<LayoutElement>().preferredHeight = 22;
            AddText(headerCopy.transform, "ROBOT RUNTIME", 32, White, FontStyle.Bold).gameObject.AddComponent<LayoutElement>().preferredHeight = 46;

            Button close = AddButton(header.transform, "ESC   CLOSE", Muted, () => SetMenuOpen(false));
            close.gameObject.AddComponent<LayoutElement>().preferredWidth = 150;

            GameObject divider = PanelObject("Accent Line", shell.transform, new Color32(39, 49, 68, 255));
            divider.AddComponent<LayoutElement>().preferredHeight = 2;

            GameObject tabsObject = NewUi("Mode Tabs", shell.transform);
            tabsObject.AddComponent<LayoutElement>().preferredHeight = 62;
            tabRow = tabsObject.GetComponent<RectTransform>();
            HorizontalLayoutGroup tabsLayout = tabsObject.AddComponent<HorizontalLayoutGroup>();
            tabsLayout.spacing = 10;
            tabsLayout.childForceExpandWidth = true;
            tabsLayout.childControlWidth = true;
            tabsLayout.childControlHeight = true;

            foreach (RobotMode mode in Enum.GetValues(typeof(RobotMode)))
            {
                RobotMode capturedMode = mode;
                ModeProfile profile = FindProfile(mode);
                Button tab = AddButton(tabRow, profile.title, profile.accent, () => SelectMode(capturedMode));
                tabs.Add(tab);
            }

            GameObject body = NewUi("Body", shell.transform);
            body.AddComponent<LayoutElement>().flexibleHeight = 1;
            HorizontalLayoutGroup bodyLayout = body.AddComponent<HorizontalLayoutGroup>();
            bodyLayout.spacing = 24;
            bodyLayout.childControlHeight = true;
            bodyLayout.childForceExpandHeight = true;
            bodyLayout.childControlWidth = true;

            GameObject info = PanelObject("Mode Information", body.transform, Panel);
            info.AddComponent<LayoutElement>().preferredWidth = 400;
            VerticalLayoutGroup infoLayout = info.AddComponent<VerticalLayoutGroup>();
            infoLayout.padding = new RectOffset(28, 28, 28, 28);
            infoLayout.spacing = 14;
            infoLayout.childForceExpandHeight = false;
            titleText = AddText(info.transform, "MODE", 28, White, FontStyle.Bold);
            titleText.gameObject.AddComponent<LayoutElement>().preferredHeight = 44;
            descriptionText = AddText(info.transform, "Description", 18, Muted, FontStyle.Normal);
            descriptionText.gameObject.AddComponent<LayoutElement>().preferredHeight = 112;
            statusText = AddText(info.transform, "● ACTIVE", 15, Color.green, FontStyle.Bold);
            statusText.gameObject.AddComponent<LayoutElement>().preferredHeight = 30;
            GameObject spacer = NewUi("Spacer", info.transform);
            spacer.AddComponent<LayoutElement>().flexibleHeight = 1;
            Text hint = AddText(info.transform, "Changes take effect when APPLY MODE is pressed.", 14, Muted, FontStyle.Italic);
            hint.gameObject.AddComponent<LayoutElement>().preferredHeight = 48;

            GameObject settings = PanelObject("Settings Panel", body.transform, Panel);
            settings.AddComponent<LayoutElement>().flexibleWidth = 1;
            VerticalLayoutGroup settingsLayout = settings.AddComponent<VerticalLayoutGroup>();
            settingsLayout.padding = new RectOffset(28, 28, 24, 24);
            settingsLayout.spacing = 12;
            settingsLayout.childForceExpandHeight = false;
            Text settingsTitle = AddText(settings.transform, "MODE SETTINGS", 17, White, FontStyle.Bold);
            settingsTitle.gameObject.AddComponent<LayoutElement>().preferredHeight = 30;

            GameObject scrollObject = NewUi("Settings Scroll", settings.transform);
            scrollObject.AddComponent<LayoutElement>().flexibleHeight = 1;
            ScrollRect scroll = scrollObject.AddComponent<ScrollRect>();
            scroll.horizontal = false;
            scroll.movementType = ScrollRect.MovementType.Clamped;
            GameObject viewport = PanelObject("Viewport", scrollObject.transform, new Color(0, 0, 0, 0));
            Stretch(viewport.GetComponent<RectTransform>());
            viewport.AddComponent<RectMask2D>();
            GameObject content = NewUi("Content", viewport.transform);
            settingsRoot = content.GetComponent<RectTransform>();
            settingsRoot.anchorMin = new Vector2(0, 1);
            settingsRoot.anchorMax = new Vector2(1, 1);
            settingsRoot.pivot = new Vector2(0.5f, 1);
            settingsRoot.sizeDelta = Vector2.zero;
            VerticalLayoutGroup contentLayout = content.AddComponent<VerticalLayoutGroup>();
            contentLayout.spacing = 10;
            contentLayout.childControlHeight = true;
            contentLayout.childForceExpandHeight = false;
            ContentSizeFitter fitter = content.AddComponent<ContentSizeFitter>();
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;
            scroll.viewport = viewport.GetComponent<RectTransform>();
            scroll.content = settingsRoot;

            GameObject footer = NewUi("Footer", shell.transform);
            footer.AddComponent<LayoutElement>().preferredHeight = 60;
            HorizontalLayoutGroup footerLayout = footer.AddComponent<HorizontalLayoutGroup>();
            footerLayout.spacing = 14;
            footerLayout.childAlignment = TextAnchor.MiddleRight;
            GameObject footerSpacer = NewUi("Spacer", footer.transform);
            footerSpacer.AddComponent<LayoutElement>().flexibleWidth = 1;
            Button cancel = AddButton(footer.transform, "CANCEL", Muted, () => SetMenuOpen(false));
            cancel.gameObject.AddComponent<LayoutElement>().preferredWidth = 140;
            applyButton = AddButton(footer.transform, "APPLY MODE", Color.cyan, ApplySelectedMode);
            applyButton.gameObject.AddComponent<LayoutElement>().preferredWidth = 230;
        }

        private void SelectMode(RobotMode mode)
        {
            selectedMode = mode;
            ModeProfile profile = FindProfile(mode);
            if (profile == null || titleText == null) return;
            titleText.text = profile.title;
            titleText.color = profile.accent;
            descriptionText.text = profile.description;
            pendingValues.Clear();
            RebuildSettings(profile);
            RefreshStatus();

            int i = 0;
            foreach (RobotMode ignored in Enum.GetValues(typeof(RobotMode)))
            {
                Image image = tabs[i].GetComponent<Image>();
                ModeProfile tabProfile = FindProfile((RobotMode)i);
                image.color = (RobotMode)i == selectedMode ? WithAlpha(tabProfile.accent, 0.34f) : PanelLight;
                tabs[i].GetComponentInChildren<Text>().color = (RobotMode)i == selectedMode ? tabProfile.accent : Muted;
                i++;
            }

            applyButton.GetComponent<Image>().color = profile.accent;
            applyButton.GetComponentInChildren<Text>().color = new Color32(8, 14, 22, 255);
        }

        private void RebuildSettings(ModeProfile profile)
        {
            for (int i = settingsRoot.childCount - 1; i >= 0; i--) Destroy(settingsRoot.GetChild(i).gameObject);
            int shown = 0;
            foreach (MemberOverride setting in profile.settings)
            {
                if (setting == null || !setting.showInHud) continue;
                shown++;
                string label = string.IsNullOrWhiteSpace(setting.displayName) ? setting.memberName : setting.displayName;
                if (setting.valueType == OverrideValueType.Boolean) AddToggleSetting(setting, label);
                else AddInputSetting(setting, label);
            }

            if (shown == 0)
            {
                Text empty = AddText(settingsRoot, "No runtime settings for this mode.\nAdd Member Overrides in the GameManager Inspector.", 16, Muted, FontStyle.Italic);
                empty.gameObject.AddComponent<LayoutElement>().preferredHeight = 70;
            }
        }

        private void AddToggleSetting(MemberOverride setting, string label)
        {
            GameObject row = PanelObject(label, settingsRoot, PanelLight);
            row.AddComponent<LayoutElement>().preferredHeight = 58;
            HorizontalLayoutGroup layout = row.AddComponent<HorizontalLayoutGroup>();
            layout.padding = new RectOffset(16, 16, 8, 8);
            layout.childAlignment = TextAnchor.MiddleLeft;
            Text text = AddText(row.transform, label, 16, White, FontStyle.Normal);
            text.gameObject.AddComponent<LayoutElement>().flexibleWidth = 1;
            Toggle toggle = AddToggle(row.transform, setting.boolValue, setting.target != null && !string.IsNullOrWhiteSpace(setting.memberName));
            toggle.onValueChanged.AddListener(value => pendingValues[setting] = value.ToString());
        }

        private void AddInputSetting(MemberOverride setting, string label)
        {
            GameObject row = PanelObject(label, settingsRoot, PanelLight);
            row.AddComponent<LayoutElement>().preferredHeight = 68;
            VerticalLayoutGroup layout = row.AddComponent<VerticalLayoutGroup>();
            layout.padding = new RectOffset(16, 16, 7, 7);
            layout.spacing = 4;
            layout.childForceExpandHeight = false;
            Text text = AddText(row.transform, label, 13, Muted, FontStyle.Bold);
            text.gameObject.AddComponent<LayoutElement>().preferredHeight = 18;
            InputField input = AddInput(row.transform, OverrideToString(setting));
            input.gameObject.AddComponent<LayoutElement>().preferredHeight = 32;
            input.interactable = setting.target != null && !string.IsNullOrWhiteSpace(setting.memberName);
            input.onValueChanged.AddListener(value => pendingValues[setting] = value);
        }

        private void CommitPendingSettings(ModeProfile profile)
        {
            foreach (KeyValuePair<MemberOverride, string> pair in pendingValues)
            {
                MemberOverride setting = pair.Key;
                string raw = pair.Value;
                try
                {
                    switch (setting.valueType)
                    {
                        case OverrideValueType.Boolean: setting.boolValue = bool.Parse(raw); break;
                        case OverrideValueType.Integer: setting.intValue = int.Parse(raw, CultureInfo.InvariantCulture); break;
                        case OverrideValueType.Float: setting.floatValue = float.Parse(raw, CultureInfo.InvariantCulture); break;
                        case OverrideValueType.String: setting.stringValue = raw; break;
                        case OverrideValueType.Vector3: setting.vector3Value = ParseVector3(raw); break;
                        case OverrideValueType.Color: setting.colorValue = ParseColor(raw); break;
                    }
                }
                catch (Exception)
                {
                    Debug.LogWarning($"Invalid value '{raw}' for {setting.memberName}; keeping the previous value.", this);
                }
            }
            pendingValues.Clear();
        }

        private static string OverrideToString(MemberOverride setting)
        {
            switch (setting.valueType)
            {
                case OverrideValueType.Integer: return setting.intValue.ToString(CultureInfo.InvariantCulture);
                case OverrideValueType.Float: return setting.floatValue.ToString(CultureInfo.InvariantCulture);
                case OverrideValueType.String: return setting.stringValue;
                case OverrideValueType.Vector3: return $"{setting.vector3Value.x}, {setting.vector3Value.y}, {setting.vector3Value.z}";
                case OverrideValueType.Color: return $"{setting.colorValue.r}, {setting.colorValue.g}, {setting.colorValue.b}, {setting.colorValue.a}";
                default: return string.Empty;
            }
        }

        private static Vector3 ParseVector3(string raw)
        {
            string[] p = raw.Split(',');
            if (p.Length != 3) throw new FormatException();
            return new Vector3(ParseFloat(p[0]), ParseFloat(p[1]), ParseFloat(p[2]));
        }

        private static Color ParseColor(string raw)
        {
            string[] p = raw.Split(',');
            if (p.Length != 3 && p.Length != 4) throw new FormatException();
            return new Color(ParseFloat(p[0]), ParseFloat(p[1]), ParseFloat(p[2]), p.Length == 4 ? ParseFloat(p[3]) : 1f);
        }

        private static float ParseFloat(string value) => float.Parse(value.Trim(), CultureInfo.InvariantCulture);

        private void RefreshStatus()
        {
            if (statusText == null) return;
            bool active = selectedMode == ActiveMode;
            statusText.text = active ? "●  CURRENTLY ACTIVE" : $"○  SWITCH FROM {ActiveMode.ToString().ToUpperInvariant()}";
            statusText.color = active ? FindProfile(selectedMode).accent : Muted;
        }

        private void EnsureEventSystem()
        {
            if (FindFirstObjectByType<EventSystem>() != null) return;
            GameObject eventSystem = new GameObject("EventSystem", typeof(EventSystem));
#if ENABLE_INPUT_SYSTEM
            eventSystem.AddComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>();
#else
            eventSystem.AddComponent<StandaloneInputModule>();
#endif
            if (persistBetweenScenes) DontDestroyOnLoad(eventSystem);
        }

        private static GameObject NewUi(string name, Transform parent)
        {
            GameObject item = new GameObject(name, typeof(RectTransform));
            item.transform.SetParent(parent, false);
            return item;
        }

        private static GameObject PanelObject(string name, Transform parent, Color color)
        {
            GameObject item = NewUi(name, parent);
            item.AddComponent<Image>().color = color;
            return item;
        }

        private Text AddText(Transform parent, string value, int size, Color color, FontStyle style)
        {
            Text text = NewUi("Text", parent).AddComponent<Text>();
            text.font = builtInFont;
            text.text = value;
            text.fontSize = size;
            text.color = color;
            text.fontStyle = style;
            text.alignment = TextAnchor.MiddleLeft;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            return text;
        }

        private Button AddButton(Transform parent, string label, Color accent, UnityAction action)
        {
            GameObject item = PanelObject(label, parent, PanelLight);
            Button button = item.AddComponent<Button>();
            button.targetGraphic = item.GetComponent<Image>();
            ColorBlock colors = button.colors;
            colors.normalColor = PanelLight;
            colors.highlightedColor = WithAlpha(accent, 0.65f);
            colors.pressedColor = accent;
            colors.selectedColor = WithAlpha(accent, 0.65f);
            button.colors = colors;
            button.onClick.AddListener(action);
            Text text = AddText(item.transform, label, 15, White, FontStyle.Bold);
            text.alignment = TextAnchor.MiddleCenter;
            Stretch(text.rectTransform);
            return button;
        }

        private InputField AddInput(Transform parent, string value)
        {
            GameObject item = PanelObject("Input", parent, Navy);
            InputField input = item.AddComponent<InputField>();
            Text text = AddText(item.transform, value, 15, White, FontStyle.Normal);
            text.rectTransform.offsetMin = new Vector2(10, 0);
            text.rectTransform.offsetMax = new Vector2(-10, 0);
            Stretch(text.rectTransform);
            input.textComponent = text;
            input.text = value;
            return input;
        }

        private Toggle AddToggle(Transform parent, bool value, bool interactable)
        {
            GameObject root = NewUi("Toggle", parent);
            root.AddComponent<LayoutElement>().preferredWidth = 54;
            Toggle toggle = root.AddComponent<Toggle>();
            GameObject background = PanelObject("Background", root.transform, Navy);
            Stretch(background.GetComponent<RectTransform>());
            GameObject check = PanelObject("Checkmark", background.transform, new Color32(82, 210, 160, 255));
            RectTransform checkRect = check.GetComponent<RectTransform>();
            checkRect.anchorMin = new Vector2(0.52f, 0.12f);
            checkRect.anchorMax = new Vector2(0.94f, 0.88f);
            checkRect.offsetMin = checkRect.offsetMax = Vector2.zero;
            toggle.targetGraphic = background.GetComponent<Image>();
            toggle.graphic = check.GetComponent<Image>();
            toggle.isOn = value;
            toggle.interactable = interactable;
            return toggle;
        }

        private static void Stretch(RectTransform rect)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
        }

        private static Color WithAlpha(Color color, float alpha)
        {
            color.a = alpha;
            return color;
        }
    }
}
