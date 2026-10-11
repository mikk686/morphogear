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

        [Header("Required references")]
        [SerializeField]
        private UnifiedKeyboardController keyboardController;

        [SerializeField]
        private ROSTelemetry telemetry;

        [Header("Startup")]
        [SerializeField]
        private RobotMode startingMode = RobotMode.Game;

        [SerializeField]
        private bool pauseGameWhileMenuIsOpen = true;

        [Header("Mode profiles")]
        [SerializeField]
        private List<ModeProfile> profiles =
            new List<ModeProfile>();

        [Header("Events")]
        public UnityEvent onBeforeModeChanged;
        public UnityEvent onModeChanged;

        public RobotMode ActiveMode { get; private set; }
        public event Action<RobotMode> ModeChanged;

        public bool IsMenuOpen =>
            menuRoot != null && menuRoot.activeSelf;

        private GameObject menuRoot;
        private RectTransform tabRoot;
        private RectTransform settingsRoot;

        private Text titleText;
        private Text descriptionText;
        private Text statusText;
        private Text errorText;

        private Font font;
        private RobotMode selectedMode;
        private float previousTimeScale = 1f;

        private readonly List<GameObject> spawnedObjects =
            new List<GameObject>();

        private static readonly Color Background =
            new Color32(12, 18, 29, 250);

        private static readonly Color Panel =
            new Color32(26, 35, 51, 255);

        private static readonly Color Row =
            new Color32(37, 48, 67, 255);

        private static readonly Color White =
            new Color32(240, 245, 252, 255);

        private static readonly Color Muted =
            new Color32(159, 173, 191, 255);

        private void Reset()
        {
            profiles = new List<ModeProfile>
            {
                MakeProfile(
                    RobotMode.Game,
                    "GAME",
                    "Standalone Unity control. No robot connection.",
                    new Color32(80, 200, 255, 255)),

                MakeProfile(
                    RobotMode.Mirror,
                    "MIRROR",
                    "Unity keyboard control. Send Unity joint angles to the robot.",
                    new Color32(173, 112, 255, 255)),

                MakeProfile(
                    RobotMode.Simulation,
                    "SIMULATION",
                    "Robot keyboard control. Receive joints and optionally publish Unity pose.",
                    new Color32(255, 170, 81, 255)),

                MakeProfile(
                    RobotMode.Visualisation,
                    "VISUALISATION",
                    "Receive robot joints. Choose a scene Transform or received robot pose for positioning.",
                    new Color32(85, 221, 164, 255))
            };

            profiles[2].showPublishPoseInMenu = true;

            profiles[3].showReceivePoseInMenu = true;
            profiles[3].showVisualisationSourceInMenu = true;
        }

        private static ModeProfile MakeProfile(
            RobotMode mode,
            string title,
            string description,
            Color accent)
        {
            return new ModeProfile
            {
                mode = mode,
                title = title,
                description = description,
                accent = accent
            };
        }

        private void Awake()
        {
            if (Instance != null && Instance != this)
            {
                Destroy(gameObject);
                return;
            }

            Instance = this;

            EnsureProfiles();

            font = Resources.GetBuiltinResource<Font>(
                "LegacyRuntime.ttf");

            BuildMenu();
            menuRoot.SetActive(false);
        }

        private void Start()
        {
            ApplyMode(startingMode);
        }

        private void Update()
        {
#if ENABLE_INPUT_SYSTEM
            bool pressed =
                Keyboard.current != null &&
                Keyboard.current.escapeKey.wasPressedThisFrame;
#else
            bool pressed = Input.GetKeyDown(KeyCode.Escape);
#endif

            if (pressed)
                SetMenuOpen(!IsMenuOpen);
        }

        private void OnDestroy()
        {
            if (Instance == this)
                Instance = null;

            if (IsMenuOpen && pauseGameWhileMenuIsOpen)
                Time.timeScale = previousTimeScale;
        }

        public void ToggleMenu()
        {
            SetMenuOpen(!IsMenuOpen);
        }

        public void SetMenuOpen(bool open)
        {
            if (menuRoot == null ||
                menuRoot.activeSelf == open)
                return;

            if (open)
            {
                selectedMode = ActiveMode;
                RefreshMenu();
            }

            menuRoot.SetActive(open);

            if (keyboardController != null)
                keyboardController.SetInputBlocked(open);

            if (!pauseGameWhileMenuIsOpen)
                return;

            if (open)
            {
                previousTimeScale = Time.timeScale;
                Time.timeScale = 0f;
            }
            else
            {
                Time.timeScale = previousTimeScale;
            }
        }

        public void ApplySelectedMode()
        {
            ModeProfile profile = FindProfile(selectedMode);

            if (profile == null)
                return;

            if (selectedMode == RobotMode.Visualisation &&
                profile.visualisationSource ==
                    TeleportPoseSource.ReceivedPoseStamped &&
                !profile.receivePose)
            {
                errorText.text =
                    "Enable RECEIVE ROBOT POSE before selecting " +
                    "ReceivedPoseStamped.";
                return;
            }

            errorText.text = "";

            if (ApplyMode(selectedMode))
                SetMenuOpen(false);
        }

        public bool ApplyMode(RobotMode mode)
        {
            ModeProfile profile = FindProfile(mode);

            if (profile == null ||
                keyboardController == null ||
                telemetry == null)
            {
                Debug.LogError(
                    "GameManager needs four profiles plus references " +
                    "to UnifiedKeyboardController and ROSTelemetry.",
                    this);

                return false;
            }

            if (mode == RobotMode.Visualisation &&
                profile.visualisationSource ==
                    TeleportPoseSource.ReceivedPoseStamped &&
                !profile.receivePose)
            {
                Debug.LogError(
                    "ReceivedPoseStamped source requires receivePose.",
                    this);

                return false;
            }

            onBeforeModeChanged?.Invoke();

            DestroySpawnedObjects();
            DisableManagedExtensions();

            // Always clear old telemetry routing first. This matters
            // when returning to Game mode.
            telemetry.publishJoints = false;
            telemetry.receiveJoints = false;
            telemetry.visualization = false;
            telemetry.poseChannelMode = PoseChannelMode.Off;
            telemetry.publishDepth = false;

            switch (mode)
            {
                case RobotMode.Game:
                    keyboardController.ConfigureManagedMode(
                        false,
                        KeyboardDestination.Unity,
                        false);

                    telemetry.enabled = false;
                    break;

                case RobotMode.Mirror:
                    keyboardController.ConfigureManagedMode(
                        true,
                        KeyboardDestination.Unity,
                        true);

                    ConfigureTelemetry(
                        profile,
                        publishJoints: true,
                        receiveJoints: false,
                        visualisation: false,
                        poseMode: PoseChannelMode.Off);

                    // Robot receives Unity angles instead of
                    // generating its own joint targets.
                    keyboardController.SetManualControl(true);
                    break;

                case RobotMode.Simulation:
                    keyboardController.ConfigureManagedMode(
                        true,
                        KeyboardDestination.Robot,
                        true);

                    ConfigureTelemetry(
                        profile,
                        publishJoints: false,
                        receiveJoints: true,
                        visualisation: false,
                        poseMode: profile.publishPose
                            ? PoseChannelMode.Publish
                            : PoseChannelMode.Off);

                    keyboardController.SetManualControl(false);
                    break;

                case RobotMode.Visualisation:
                    keyboardController.ConfigureManagedMode(
                        true,
                        KeyboardDestination.Robot,
                        false);

                    ConfigureTelemetry(
                        profile,
                        publishJoints: false,
                        receiveJoints: true,
                        visualisation: true,
                        poseMode: profile.receivePose
                            ? PoseChannelMode.Receive
                            : PoseChannelMode.Off);

                    keyboardController.SetManualControl(false);
                    break;
            }

            ApplyExtensionSettings(profile);
            EnableManagedExtensions(profile);
            SpawnPrefabs(profile);

            ActiveMode = mode;
            selectedMode = mode;

            RefreshMenu();

            onModeChanged?.Invoke();
            ModeChanged?.Invoke(mode);

            Debug.Log($"Robot mode: {mode}", this);
            return true;
        }

        private void ConfigureTelemetry(
            ModeProfile profile,
            bool publishJoints,
            bool receiveJoints,
            bool visualisation,
            PoseChannelMode poseMode)
        {
            telemetry.publishJoints = publishJoints;
            telemetry.receiveJoints = receiveJoints;
            telemetry.visualization = visualisation;
            telemetry.poseChannelMode = poseMode;

            telemetry.teleportPoseSource =
                profile.visualisationSource;

            telemetry.publishDepth = profile.publishDepth;
            telemetry.enabled = true;
        }

        private void EnsureProfiles()
        {
            if (profiles == null)
                profiles = new List<ModeProfile>();

            foreach (RobotMode mode in
                Enum.GetValues(typeof(RobotMode)))
            {
                if (FindProfile(mode) == null)
                {
                    profiles.Add(MakeProfile(
                        mode,
                        mode.ToString().ToUpperInvariant(),
                        "Configure this profile in the Inspector.",
                        Color.cyan));
                }
            }
        }

        private ModeProfile FindProfile(RobotMode mode)
        {
            return profiles.Find(
                p => p != null && p.mode == mode);
        }

        private void DisableManagedExtensions()
        {
            var visited = new HashSet<Behaviour>();

            foreach (ModeProfile profile in profiles)
            {
                if (profile == null)
                    continue;

                foreach (ComponentToggle item in
                    profile.sceneComponents)
                {
                    if (item == null ||
                        item.component == null ||
                        !visited.Add(item.component))
                        continue;

                    // These are managed by ApplyMode itself.
                    if (item.component == keyboardController ||
                        item.component == telemetry)
                        continue;

                    item.component.enabled = false;
                }
            }
        }

        private void EnableManagedExtensions(ModeProfile profile)
        {
            foreach (ComponentToggle item in
                profile.sceneComponents)
            {
                if (item == null ||
                    item.component == null ||
                    item.component == keyboardController ||
                    item.component == telemetry)
                    continue;

                item.component.enabled = item.enabled;
            }
        }

        private void ApplyExtensionSettings(ModeProfile profile)
        {
            const BindingFlags flags =
                BindingFlags.Instance | BindingFlags.Public;

            foreach (MemberOverride setting in profile.settings)
            {
                if (setting == null ||
                    setting.target == null ||
                    string.IsNullOrWhiteSpace(setting.memberName))
                    continue;

                // Do not allow a generic override to undo core routing.
                if (IsProtectedRoutingMember(setting))
                {
                    Debug.LogWarning(
                        $"Routing member '{setting.memberName}' is " +
                        "controlled by GameManager; override skipped.",
                        setting.target);

                    continue;
                }

                Type type = setting.target.GetType();

                FieldInfo field =
                    type.GetField(setting.memberName, flags);

                PropertyInfo property =
                    type.GetProperty(setting.memberName, flags);

                Type destination =
                    field != null
                        ? field.FieldType
                        : property != null && property.CanWrite
                            ? property.PropertyType
                            : null;

                if (destination == null)
                {
                    Debug.LogWarning(
                        $"No writable public member " +
                        $"{type.Name}.{setting.memberName}.",
                        setting.target);

                    continue;
                }

                try
                {
                    object value =
                        ConvertSetting(setting, destination);

                    if (field != null)
                        field.SetValue(setting.target, value);
                    else
                        property.SetValue(setting.target, value);
                }
                catch (Exception ex)
                {
                    Debug.LogError(
                        $"Failed to apply {type.Name}." +
                        $"{setting.memberName}: {ex.Message}",
                        setting.target);
                }
            }
        }

        private bool IsProtectedRoutingMember(
            MemberOverride setting)
        {
            if (setting.target == keyboardController)
            {
                return setting.memberName ==
                           "CurrentDestination";
            }

            if (setting.target != telemetry)
                return false;

            return setting.memberName == "publishJoints" ||
                   setting.memberName == "receiveJoints" ||
                   setting.memberName == "visualization" ||
                   setting.memberName == "poseChannelMode" ||
                   setting.memberName == "teleportPoseSource" ||
                   setting.memberName == "publishDepth";
        }

        private static object ConvertSetting(
            MemberOverride setting,
            Type destination)
        {
            if (destination.IsEnum)
                return Enum.Parse(
                    destination,
                    setting.stringValue,
                    true);

            object value;

            switch (setting.valueType)
            {
                case OverrideValueType.Boolean:
                    value = setting.boolValue;
                    break;

                case OverrideValueType.Integer:
                    value = setting.intValue;
                    break;

                case OverrideValueType.Float:
                    value = setting.floatValue;
                    break;

                case OverrideValueType.Vector3:
                    value = setting.vector3Value;
                    break;

                case OverrideValueType.Color:
                    value = setting.colorValue;
                    break;

                default:
                    value = setting.stringValue;
                    break;
            }

            if (destination.IsInstanceOfType(value))
                return value;

            return Convert.ChangeType(
                value,
                destination,
                CultureInfo.InvariantCulture);
        }

        private void SpawnPrefabs(ModeProfile profile)
        {
            foreach (GameObject prefab in
                profile.startupPrefabs)
            {
                if (prefab == null)
                    continue;

                GameObject instance =
                    Instantiate(prefab, transform);

                spawnedObjects.Add(instance);

                foreach (MonoBehaviour behaviour in
                    instance.GetComponentsInChildren
                        <MonoBehaviour>(true))
                {
                    if (behaviour is IModeInitializable hook)
                        hook.InitializeForMode(
                            profile.mode,
                            this);
                }
            }
        }

        private void DestroySpawnedObjects()
        {
            foreach (GameObject item in spawnedObjects)
            {
                if (item != null)
                    Destroy(item);
            }

            spawnedObjects.Clear();
        }

        // ----------------------------------------------------------
        // Runtime menu
        // ----------------------------------------------------------

        //private void BuildMenu()
        //{
        //    EnsureEventSystem();

        //    GameObject canvasObject =
        //        NewUI("Robot Mode Canvas", transform);

        //    Canvas canvas =
        //        canvasObject.AddComponent<Canvas>();

        //    canvas.renderMode =
        //        RenderMode.ScreenSpaceOverlay;

        //    canvas.sortingOrder = 500;

        //    canvasObject.AddComponent<GraphicRaycaster>();

        //    CanvasScaler scaler =
        //        canvasObject.AddComponent<CanvasScaler>();

        //    scaler.uiScaleMode =
        //        CanvasScaler.ScaleMode.ScaleWithScreenSize;

        //    scaler.referenceResolution =
        //        new Vector2(1920, 1080);

        //    menuRoot =
        //        PanelObject("Menu", canvasObject.transform,
        //            new Color(0f, 0f, 0f, 0.78f));

        //    Stretch(menuRoot.GetComponent<RectTransform>());

        //    GameObject shell =
        //        PanelObject("Shell", menuRoot.transform,
        //            Background);

        //    RectTransform shellRect =
        //        shell.GetComponent<RectTransform>();

        //    shellRect.anchorMin =
        //        shellRect.anchorMax =
        //            new Vector2(0.5f, 0.5f);

        //    shellRect.sizeDelta =
        //        new Vector2(1080f, 720f);

        //    VerticalLayoutGroup shellLayout =
        //        shell.AddComponent<VerticalLayoutGroup>();

        //    shellLayout.padding =
        //        new RectOffset(34, 34, 28, 28);

        //    shellLayout.spacing = 15;
        //    shellLayout.childForceExpandHeight = false;

        //    AddText(shell.transform, "ROBOT RUNTIME", 30, White)
        //        .gameObject
        //        .AddComponent<LayoutElement>()
        //        .preferredHeight = 48;

        //    GameObject tabs =
        //        NewUI("Mode Tabs", shell.transform);

        //    tabs.AddComponent<LayoutElement>()
        //        .preferredHeight = 28;

        //    tabRoot =
        //        tabs.GetComponent<RectTransform>();

        //    HorizontalLayoutGroup tabLayout =
        //        tabs.AddComponent<HorizontalLayoutGroup>();

        //    tabLayout.spacing = 9;
        //    tabLayout.childControlWidth = true;
        //    tabLayout.childForceExpandWidth = true;
        //    tabLayout.childControlHeight = true;

        //    foreach (RobotMode mode in
        //        Enum.GetValues(typeof(RobotMode)))
        //    {
        //        RobotMode selected = mode;

        //        AddButton(
        //            tabRoot,
        //            FindProfile(mode).title,
        //            () =>
        //            {
        //                selectedMode = selected;
        //                RefreshMenu();
        //            });
        //    }

        //    GameObject info =
        //        PanelObject(
        //            "Mode Description",
        //            shell.transform,
        //            Panel);

        //    info.AddComponent<LayoutElement>()
        //        .preferredHeight = 70;

        //    VerticalLayoutGroup infoLayout =
        //        info.AddComponent<VerticalLayoutGroup>();

        //    infoLayout.padding =
        //        new RectOffset(18, 18, 12, 12);

        //    infoLayout.childForceExpandHeight = false;

        //    titleText =
        //        AddText(info.transform, "", 26, White);

        //    titleText.gameObject
        //        .AddComponent<LayoutElement>()
        //        .preferredHeight = 40;

        //    descriptionText =
        //        AddText(info.transform, "", 17, Muted);

        //    descriptionText.gameObject
        //        .AddComponent<LayoutElement>()
        //        .preferredHeight = 53;

        //    statusText =
        //        AddText(info.transform, "", 15, White);

        //    statusText.gameObject
        //        .AddComponent<LayoutElement>()
        //        .preferredHeight = 25;

        //    GameObject scrollObject =
        //        PanelObject(
        //            "Settings Panel",
        //            shell.transform,
        //            Panel);

        //    scrollObject.AddComponent<LayoutElement>()
        //        .flexibleHeight = 1;

        //    ScrollRect scroll =
        //        scrollObject.AddComponent<ScrollRect>();

        //    scroll.horizontal = false;

        //    GameObject viewport =
        //        NewUI("Viewport", scrollObject.transform);

        //    Stretch(viewport.GetComponent<RectTransform>());

        //    viewport.AddComponent<RectMask2D>();

        //    GameObject content =
        //        NewUI("Settings", viewport.transform);

        //    settingsRoot =
        //        content.GetComponent<RectTransform>();

        //    settingsRoot.anchorMin =
        //        new Vector2(0f, 1f);

        //    settingsRoot.anchorMax =
        //        new Vector2(1f, 1f);

        //    settingsRoot.pivot =
        //        new Vector2(0.5f, 1f);

        //    settingsRoot.sizeDelta = Vector2.zero;

        //    VerticalLayoutGroup settingsLayout =
        //        content.AddComponent<VerticalLayoutGroup>();

        //    settingsLayout.padding =
        //        new RectOffset(18, 18, 16, 16);

        //    settingsLayout.spacing = 9;
        //    settingsLayout.childForceExpandHeight = false;

        //    ContentSizeFitter fitter =
        //        content.AddComponent<ContentSizeFitter>();

        //    fitter.verticalFit =
        //        ContentSizeFitter.FitMode.PreferredSize;

        //    scroll.viewport =
        //        viewport.GetComponent<RectTransform>();

        //    scroll.content = settingsRoot;

        //    errorText =
        //        AddText(
        //            shell.transform,
        //            "",
        //            15,
        //            new Color32(255, 125, 125, 255));

        //    errorText.gameObject
        //        .AddComponent<LayoutElement>()
        //        .preferredHeight = 24;

        //    GameObject footer =
        //        NewUI("Footer", shell.transform);

        //    footer.AddComponent<LayoutElement>()
        //        .preferredHeight = 54;

        //    HorizontalLayoutGroup footerLayout =
        //        footer.AddComponent<HorizontalLayoutGroup>();

        //    footerLayout.spacing = 12;
        //    footerLayout.childControlWidth = true;
        //    footerLayout.childForceExpandWidth = true;
        //    footerLayout.childControlHeight = true;

        //    AddButton(
        //        footer.transform,
        //        "CANCEL",
        //        () => SetMenuOpen(false));

        //    AddButton(
        //        footer.transform,
        //        "APPLY MODE",
        //        ApplySelectedMode);
        //}

        private void BuildMenu()
        {
            EnsureEventSystem();

            GameObject canvasObject =
                NewUI("Robot Mode Canvas", transform);

            Canvas canvas = canvasObject.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = 500;

            canvasObject.AddComponent<GraphicRaycaster>();

            CanvasScaler scaler =
                canvasObject.AddComponent<CanvasScaler>();

            scaler.uiScaleMode =
                CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);

            menuRoot = PanelObject(
                "Menu",
                canvasObject.transform,
                new Color(0f, 0f, 0f, 0.78f));

            Stretch(menuRoot.GetComponent<RectTransform>());

            // Main centered window
            GameObject shell =
                PanelObject("Shell", menuRoot.transform, Background);

            RectTransform shellRect =
                shell.GetComponent<RectTransform>();

            shellRect.anchorMin = new Vector2(0.5f, 0.5f);
            shellRect.anchorMax = new Vector2(0.5f, 0.5f);
            shellRect.pivot = new Vector2(0.5f, 0.5f);
            shellRect.sizeDelta = new Vector2(1080f, 650f);

            VerticalLayoutGroup shellLayout =
                shell.AddComponent<VerticalLayoutGroup>();

            shellLayout.padding = new RectOffset(28, 28, 22, 22);
            shellLayout.spacing = 12;
            shellLayout.childControlHeight = true;
            shellLayout.childForceExpandHeight = false;

            // Title
            AddText(shell.transform, "ROBOT RUNTIME", 28, White)
                .gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 40;

            // Compact mode buttons
            GameObject tabs = NewUI("Mode Tabs", shell.transform);

            tabs.AddComponent<LayoutElement>()
                .preferredHeight = 40;

            tabRoot = tabs.GetComponent<RectTransform>();

            HorizontalLayoutGroup tabLayout =
                tabs.AddComponent<HorizontalLayoutGroup>();

            tabLayout.spacing = 8;
            tabLayout.childControlWidth = true;
            tabLayout.childForceExpandWidth = true;
            tabLayout.childControlHeight = true;
            tabLayout.childForceExpandHeight = true;

            foreach (RobotMode mode in
                Enum.GetValues(typeof(RobotMode)))
            {
                RobotMode capturedMode = mode;

                AddButton(
                    tabRoot,
                    FindProfile(mode).title,
                    () =>
                    {
                        selectedMode = capturedMode;
                        RefreshMenu();
                    });
            }

            // Body: information LEFT, scrollable settings RIGHT
            GameObject body = NewUI("Body", shell.transform);

            body.AddComponent<LayoutElement>()
                .flexibleHeight = 1;

            HorizontalLayoutGroup bodyLayout =
                body.AddComponent<HorizontalLayoutGroup>();

            bodyLayout.spacing = 16;
            bodyLayout.childControlWidth = true;
            bodyLayout.childControlHeight = true;
            bodyLayout.childForceExpandWidth = false;
            bodyLayout.childForceExpandHeight = true;

            // LEFT: mode information
            GameObject info =
                PanelObject("Mode Information", body.transform, Panel);

            info.AddComponent<LayoutElement>()
                .preferredWidth = 345;

            VerticalLayoutGroup infoLayout =
                info.AddComponent<VerticalLayoutGroup>();

            infoLayout.padding =
                new RectOffset(20, 20, 18, 18);

            infoLayout.spacing = 12;
            infoLayout.childControlHeight = true;
            infoLayout.childForceExpandHeight = false;

            titleText =
                AddText(info.transform, "", 26, White);

            titleText.gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 42;

            descriptionText =
                AddText(info.transform, "", 17, Muted);

            descriptionText.horizontalOverflow =
                HorizontalWrapMode.Wrap;

            descriptionText.verticalOverflow =
                VerticalWrapMode.Overflow;

            descriptionText.gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 130;

            statusText =
                AddText(info.transform, "", 15, White);

            statusText.gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 32;

            // RIGHT: scrollable options
            GameObject settingsPanel =
                PanelObject("Settings Panel", body.transform, Panel);

            settingsPanel.AddComponent<LayoutElement>()
                .flexibleWidth = 1;

            VerticalLayoutGroup panelLayout =
                settingsPanel.AddComponent<VerticalLayoutGroup>();

            panelLayout.padding =
                new RectOffset(14, 14, 14, 14);

            panelLayout.spacing = 10;
            panelLayout.childControlHeight = true;
            panelLayout.childForceExpandHeight = false;

            AddText(settingsPanel.transform, "MODE SETTINGS", 17, White)
                .gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 30;

            GameObject scrollObject =
                NewUI("Settings Scroll", settingsPanel.transform);

            scrollObject.AddComponent<LayoutElement>()
                .flexibleHeight = 1;

            ScrollRect scroll =
                scrollObject.AddComponent<ScrollRect>();

            scroll.horizontal = false;
            scroll.vertical = true;
            scroll.movementType = ScrollRect.MovementType.Clamped;

            GameObject viewport =
                NewUI("Viewport", scrollObject.transform);

            Stretch(viewport.GetComponent<RectTransform>());
            viewport.AddComponent<RectMask2D>();

            GameObject content =
                NewUI("Settings", viewport.transform);

            settingsRoot = content.GetComponent<RectTransform>();

            settingsRoot.anchorMin = new Vector2(0f, 1f);
            settingsRoot.anchorMax = new Vector2(1f, 1f);
            settingsRoot.pivot = new Vector2(0.5f, 1f);
            settingsRoot.sizeDelta = Vector2.zero;

            VerticalLayoutGroup settingsLayout =
                content.AddComponent<VerticalLayoutGroup>();

            settingsLayout.padding =
                new RectOffset(4, 4, 4, 4);

            settingsLayout.spacing = 9;
            settingsLayout.childControlHeight = true;
            settingsLayout.childForceExpandHeight = false;

            ContentSizeFitter fitter =
                content.AddComponent<ContentSizeFitter>();

            fitter.verticalFit =
                ContentSizeFitter.FitMode.PreferredSize;

            scroll.viewport =
                viewport.GetComponent<RectTransform>();

            scroll.content = settingsRoot;

            // Validation messages
            errorText =
                AddText(
                    shell.transform,
                    "",
                    14,
                    new Color32(255, 125, 125, 255));

            errorText.gameObject
                .AddComponent<LayoutElement>()
                .preferredHeight = 22;

            // Compact Cancel / Apply buttons
            GameObject footer =
                NewUI("Footer", shell.transform);

            footer.AddComponent<LayoutElement>()
                .preferredHeight = 40;

            HorizontalLayoutGroup footerLayout =
                footer.AddComponent<HorizontalLayoutGroup>();

            footerLayout.spacing = 10;
            footerLayout.childControlWidth = true;
            footerLayout.childControlHeight = true;
            footerLayout.childForceExpandWidth = false;
            footerLayout.childForceExpandHeight = true;
            footerLayout.childAlignment = TextAnchor.MiddleRight;

            // Spacer keeps the buttons on the right.
            GameObject spacer = NewUI("Spacer", footer.transform);
            spacer.AddComponent<LayoutElement>()
                .flexibleWidth = 1;

            Button cancel =
                AddButton(
                    footer.transform,
                    "CANCEL",
                    () => SetMenuOpen(false));

            cancel.gameObject.AddComponent<LayoutElement>()
                .preferredWidth = 125;

            Button apply =
                AddButton(
                    footer.transform,
                    "APPLY MODE",
                    ApplySelectedMode);

            apply.gameObject.AddComponent<LayoutElement>()
                .preferredWidth = 155;
        }

        private void RefreshMenu()
        {
            if (menuRoot == null)
                return;

            ModeProfile profile =
                FindProfile(selectedMode);

            titleText.text = profile.title;
            titleText.color = profile.accent;

            descriptionText.text =
                profile.description;

            statusText.text =
                selectedMode == ActiveMode
                    ? "● CURRENTLY ACTIVE"
                    : $"○ SWITCH FROM {ActiveMode}";

            statusText.color =
                selectedMode == ActiveMode
                    ? profile.accent
                    : Muted;

            int index = 0;

            foreach (RobotMode mode in
                Enum.GetValues(typeof(RobotMode)))
            {
                Image image =
                    tabRoot.GetChild(index)
                        .GetComponent<Image>();

                image.color =
                    mode == selectedMode
                        ? FindProfile(mode).accent
                        : Row;

                index++;
            }

            for (int i = settingsRoot.childCount - 1;
                 i >= 0;
                 i--)
            {
                Destroy(
                    settingsRoot.GetChild(i).gameObject);
            }

            if (selectedMode == RobotMode.Simulation &&
                profile.showPublishPoseInMenu)
            {
                AddBoolSetting(
                    "Publish simulated pose",
                    profile.publishPose,
                    value => profile.publishPose = value);
            }

            if (selectedMode == RobotMode.Visualisation)
            {
                if (profile.showReceivePoseInMenu)
                {
                    AddBoolSetting(
                        "Receive robot pose",
                        profile.receivePose,
                        value => profile.receivePose = value);
                }

                if (profile.showVisualisationSourceInMenu)
                {
                    AddCycleSetting(
                        "Visualisation position source",
                        profile.visualisationSource.ToString(),
                        () =>
                        {
                            profile.visualisationSource =
                                profile.visualisationSource ==
                                    TeleportPoseSource.PoseTransform
                                    ? TeleportPoseSource
                                        .ReceivedPoseStamped
                                    : TeleportPoseSource
                                        .PoseTransform;

                            RefreshMenu();
                        });
                }
            }

            if (selectedMode != RobotMode.Game &&
                profile.showPublishDepthInMenu)
            {
                AddBoolSetting(
                    "Publish depth",
                    profile.publishDepth,
                    value => profile.publishDepth = value);
            }

            foreach (MemberOverride setting in
                profile.settings)
            {
                if (setting == null ||
                    !setting.showInHud ||
                    IsProtectedRoutingMember(setting))
                    continue;

                AddMemberSetting(setting);
            }

            if (settingsRoot.childCount == 0)
            {
                AddText(
                    settingsRoot,
                    "No runtime settings exposed for this mode.",
                    16,
                    Muted)
                    .gameObject
                    .AddComponent<LayoutElement>()
                    .preferredHeight = 50;
            }
        }

        private void AddMemberSetting(
            MemberOverride setting)
        {
            string label =
                string.IsNullOrWhiteSpace(
                    setting.displayName)
                    ? setting.memberName
                    : setting.displayName;

            if (setting.valueType ==
                OverrideValueType.Boolean)
            {
                AddBoolSetting(
                    label,
                    setting.boolValue,
                    value => setting.boolValue = value);

                return;
            }

            if (setting.valueType ==
                OverrideValueType.Enum)
            {
                Type enumType =
                    GetMemberType(setting);

                if (enumType == null ||
                    !enumType.IsEnum)
                    return;

                string[] values =
                    Enum.GetNames(enumType);

                if (values.Length == 0)
                    return;

                if (Array.IndexOf(
                    values,
                    setting.stringValue) < 0)
                {
                    setting.stringValue = values[0];
                }

                AddCycleSetting(
                    label,
                    setting.stringValue,
                    () =>
                    {
                        int current =
                            Array.IndexOf(
                                values,
                                setting.stringValue);

                        setting.stringValue =
                            values[
                                (current + 1) %
                                values.Length];

                        RefreshMenu();
                    });

                return;
            }

            string current =
                SettingAsString(setting);

            AddTextSetting(
                label,
                current,
                raw => UpdateSetting(setting, raw));
        }

        private static Type GetMemberType(
            MemberOverride setting)
        {
            if (setting.target == null)
                return null;

            const BindingFlags flags =
                BindingFlags.Instance |
                BindingFlags.Public;

            Type type = setting.target.GetType();

            return type.GetField(
                       setting.memberName,
                       flags)?.FieldType ??
                   type.GetProperty(
                       setting.memberName,
                       flags)?.PropertyType;
        }

        private static string SettingAsString(
            MemberOverride setting)
        {
            switch (setting.valueType)
            {
                case OverrideValueType.Integer:
                    return setting.intValue.ToString(
                        CultureInfo.InvariantCulture);

                case OverrideValueType.Float:
                    return setting.floatValue.ToString(
                        CultureInfo.InvariantCulture);

                case OverrideValueType.Vector3:
                    return string.Format(
                        CultureInfo.InvariantCulture,
                        "{0}, {1}, {2}",
                        setting.vector3Value.x,
                        setting.vector3Value.y,
                        setting.vector3Value.z);

                case OverrideValueType.Color:
                    return string.Format(
                        CultureInfo.InvariantCulture,
                        "{0}, {1}, {2}, {3}",
                        setting.colorValue.r,
                        setting.colorValue.g,
                        setting.colorValue.b,
                        setting.colorValue.a);

                default:
                    return setting.stringValue ?? "";
            }
        }

        private void UpdateSetting(
            MemberOverride setting,
            string raw)
        {
            try
            {
                switch (setting.valueType)
                {
                    case OverrideValueType.Integer:
                        setting.intValue = int.Parse(
                            raw,
                            CultureInfo.InvariantCulture);
                        break;

                    case OverrideValueType.Float:
                        setting.floatValue = float.Parse(
                            raw,
                            CultureInfo.InvariantCulture);
                        break;

                    case OverrideValueType.String:
                        setting.stringValue = raw;
                        break;

                    case OverrideValueType.Vector3:
                        {
                            string[] p = raw.Split(',');

                            if (p.Length != 3)
                                throw new FormatException();

                            setting.vector3Value =
                                new Vector3(
                                    ParseFloat(p[0]),
                                    ParseFloat(p[1]),
                                    ParseFloat(p[2]));

                            break;
                        }

                    case OverrideValueType.Color:
                        {
                            string[] p = raw.Split(',');

                            if (p.Length != 3 &&
                                p.Length != 4)
                                throw new FormatException();

                            setting.colorValue =
                                new Color(
                                    ParseFloat(p[0]),
                                    ParseFloat(p[1]),
                                    ParseFloat(p[2]),
                                    p.Length == 4
                                        ? ParseFloat(p[3])
                                        : 1f);

                            break;
                        }
                }

                errorText.text = "";
            }
            catch (Exception)
            {
                errorText.text =
                    $"Invalid value for {setting.memberName}.";
            }
        }

        private static float ParseFloat(string value)
        {
            return float.Parse(
                value.Trim(),
                CultureInfo.InvariantCulture);
        }

        private void AddBoolSetting(
            string label,
            bool initial,
            Action<bool> changed)
        {
            GameObject row =
                MakeRow(label, 55f);

            Text text =
                AddText(
                    row.transform,
                    label,
                    16,
                    White);

            text.gameObject
                .AddComponent<LayoutElement>()
                .flexibleWidth = 1;

            GameObject box =
                PanelObject(
                    "Toggle",
                    row.transform,
                    Background);

            box.AddComponent<LayoutElement>()
                .preferredWidth = 58;

            Toggle toggle =
                box.AddComponent<Toggle>();

            GameObject check =
                PanelObject(
                    "Check",
                    box.transform,
                    new Color32(
                        86, 225, 170, 255));

            RectTransform rect =
                check.GetComponent<RectTransform>();

            rect.anchorMin =
                new Vector2(0.25f, 0.2f);

            rect.anchorMax =
                new Vector2(0.75f, 0.8f);

            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;

            toggle.targetGraphic =
                box.GetComponent<Image>();

            toggle.graphic =
                check.GetComponent<Image>();

            toggle.isOn = initial;

            toggle.onValueChanged.AddListener(
                value => changed(value));
        }

        private void AddCycleSetting(
            string label,
            string current,
            UnityAction next)
        {
            GameObject row =
                MakeRow(label, 55f);

            AddText(
                row.transform,
                label,
                16,
                White)
                .gameObject
                .AddComponent<LayoutElement>()
                .flexibleWidth = 1;

            Button button =
                AddButton(
                    row.transform,
                    current + "  ▸",
                    next);

            button.gameObject
                .AddComponent<LayoutElement>()
                .preferredWidth = 255;
        }

        private void AddTextSetting(
            string label,
            string current,
            Action<string> changed)
        {
            GameObject row =
                MakeRow(label, 66f);

            AddText(
                row.transform,
                label,
                15,
                White)
                .gameObject
                .AddComponent<LayoutElement>()
                .flexibleWidth = 1;

            GameObject inputObject =
                PanelObject(
                    "Value",
                    row.transform,
                    Background);

            inputObject.AddComponent<LayoutElement>()
                .preferredWidth = 275;

            InputField input =
                inputObject.AddComponent<InputField>();

            Text text =
                AddText(
                    inputObject.transform,
                    "",
                    15,
                    White);

            Stretch(text.rectTransform);

            text.rectTransform.offsetMin =
                new Vector2(10f, 0f);

            text.rectTransform.offsetMax =
                new Vector2(-10f, 0f);

            input.textComponent = text;
            input.text = current;

            input.onEndEdit.AddListener(
                value => changed(value));
        }

        private GameObject MakeRow(
            string label,
            float height)
        {
            GameObject row =
                PanelObject(
                    label,
                    settingsRoot,
                    Row);

            row.AddComponent<LayoutElement>()
                .preferredHeight = height;

            HorizontalLayoutGroup layout =
                row.AddComponent<HorizontalLayoutGroup>();

            layout.padding =
                new RectOffset(16, 16, 7, 7);

            layout.spacing = 12;
            layout.childControlWidth = true;
            layout.childControlHeight = true;

            return row;
        }

        private void EnsureEventSystem()
        {
            if (FindFirstObjectByType<EventSystem>() != null)
                return;

            GameObject system =
                new GameObject(
                    "EventSystem",
                    typeof(EventSystem));

#if ENABLE_INPUT_SYSTEM
            system.AddComponent<
                UnityEngine.InputSystem.UI
                    .InputSystemUIInputModule>();
#else
            system.AddComponent<
                StandaloneInputModule>();
#endif
        }

        private static GameObject NewUI(
            string name,
            Transform parent)
        {
            GameObject item =
                new GameObject(
                    name,
                    typeof(RectTransform));

            item.transform.SetParent(
                parent,
                false);

            return item;
        }

        private static GameObject PanelObject(
            string name,
            Transform parent,
            Color color)
        {
            GameObject item =
                NewUI(name, parent);

            item.AddComponent<Image>().color =
                color;

            return item;
        }

        private Text AddText(
            Transform parent,
            string content,
            int size,
            Color color)
        {
            Text text =
                NewUI("Text", parent)
                    .AddComponent<Text>();

            text.font = font;
            text.text = content;
            text.fontSize = size;
            text.color = color;
            text.alignment =
                TextAnchor.MiddleLeft;

            return text;
        }

        private Button AddButton(
            Transform parent,
            string label,
            UnityAction action)
        {
            GameObject item =
                PanelObject(
                    label,
                    parent,
                    Row);

            Button button =
                item.AddComponent<Button>();

            button.targetGraphic =
                item.GetComponent<Image>();

            button.onClick.AddListener(action);

            Text text =
                AddText(
                    item.transform,
                    label,
                    15,
                    White);

            text.alignment =
                TextAnchor.MiddleCenter;

            Stretch(text.rectTransform);

            return button;
        }

        private static void Stretch(
            RectTransform rect)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = Vector2.zero;
            rect.offsetMax = Vector2.zero;
        }
    }
}