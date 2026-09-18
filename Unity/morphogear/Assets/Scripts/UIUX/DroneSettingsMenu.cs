// DroneSettingsMenu.cs
// Runtime settings window (TextMeshPro), built from the DroneSettings registry.
// Left = category tabs, right = scrollable widget rows, bottom = preset Save/Load/Defaults.
//
// Drop this component on any GameObject. Press TAB to open.
using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace DroneLab
{
    [DisallowMultipleComponent]
    public partial class DroneSettingsMenu : MonoBehaviour
    {
        public static DroneSettingsMenu Instance { get; private set; }

        [Header("Behaviour")]
        public KeyCode toggleKey = KeyCode.Tab;
        [Tooltip("Set Time.timeScale = 0 while the menu is open.")]
        public bool pauseWhileOpen = true;
        public bool openOnStart = false;
        [Tooltip("Scan the scene for [DroneSetting] fields on Start.")]
        public bool autoBindSceneOnStart = true;
        public bool loadPresetOnStart = false;
        [Tooltip("Refresh interval for live Readout rows, seconds.")]
        public float readoutInterval = 0.15f;

        [Header("Layout")]
        public Vector2 windowSize = new Vector2(1080f, 640f);
        public float labelWidth = 215f;
        public float fieldWidth = 96f;
        public int sortingOrder = 500;

        public bool IsOpen { get; private set; }
        public event Action<bool> Toggled;

        private Canvas _canvas;
        private GameObject _root;
        private RectTransform _tabsPanel;
        private RectTransform _content;
        private ScrollRect _scroll;
        private TextMeshProUGUI _statusText;
        private TMP_InputField _presetField;
        private string _activeCategory;
        private float _savedTimeScale = 1f;
        private float _readoutTimer;
        private bool _dirty;

        private readonly List<Action> _refreshers = new List<Action>();
        private readonly List<Action> _readouts = new List<Action>();
        private readonly Dictionary<string, Button> _tabButtons = new Dictionary<string, Button>();

        // ------------------------------------------------------------------ unity
        private void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(this); return; }
            Instance = this;
        }

        private void Start()
        {
            if (autoBindSceneOnStart)
            {
                int n = DroneSettings.BindScene();
                if (n > 0) Debug.Log($"[DroneSettingsMenu] Auto-bound {n} [DroneSetting] fields.");
            }
            if (loadPresetOnStart) DroneSettings.LoadPreset();

            BuildWindow();
            DroneSettings.ListChanged += OnListChanged;
            SetOpen(openOnStart);
        }

        private void OnDestroy()
        {
            DroneSettings.ListChanged -= OnListChanged;
            if (Instance == this) Instance = null;
            if (pauseWhileOpen && IsOpen) Time.timeScale = _savedTimeScale;
        }

        private void Update()
        {
            if (WasTogglePressed()) Toggle();

            if (_dirty)
            {
                _dirty = false;
                RebuildTabs();
            }

            if (!IsOpen || _readouts.Count == 0) return;
            _readoutTimer += Time.unscaledDeltaTime;
            if (_readoutTimer < readoutInterval) return;
            _readoutTimer = 0f;
            for (int i = 0; i < _readouts.Count; i++)
            {
                try { _readouts[i]?.Invoke(); } catch { }
            }
        }

        private void OnListChanged() => _dirty = true;

        private bool WasTogglePressed()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb == null) return false;
            var key = KeyCodeToKey(toggleKey);
            return kb[key].wasPressedThisFrame;
#elif ENABLE_LEGACY_INPUT_MANAGER
            return Input.GetKeyDown(toggleKey);
#else
            return false;
#endif
        }

#if ENABLE_INPUT_SYSTEM
        private static Key KeyCodeToKey(KeyCode k)
        {
            switch (k)
            {
                case KeyCode.Tab: return Key.Tab;
                case KeyCode.Escape: return Key.Escape;
                case KeyCode.BackQuote: return Key.Backquote;
                case KeyCode.F1: return Key.F1;
                case KeyCode.F2: return Key.F2;
                case KeyCode.F10: return Key.F10;
                case KeyCode.M: return Key.M;
                case KeyCode.P: return Key.P;
                default: return Key.Tab;
            }
        }
#endif

        // ------------------------------------------------------------------ open / close
        public void Toggle() => SetOpen(!IsOpen);
        public void Open() => SetOpen(true);
        public void Close() => SetOpen(false);

        public void SetOpen(bool open)
        {
            IsOpen = open;
            if (_root != null) _root.SetActive(open);

            if (pauseWhileOpen)
            {
                if (open) { _savedTimeScale = Time.timeScale; Time.timeScale = 0f; }
                else Time.timeScale = _savedTimeScale <= 0f ? 1f : _savedTimeScale;
            }

            if (open) { RefreshAll(); Status(""); }
            Toggled?.Invoke(open);
        }

        // ------------------------------------------------------------------ window
        private void BuildWindow()
        {
            EnsureEventSystem();

            var canvasGo = new GameObject("DroneSettings Canvas",
                typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            canvasGo.transform.SetParent(transform, false);
            _canvas = canvasGo.GetComponent<Canvas>();
            _canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            _canvas.sortingOrder = sortingOrder;
            var scaler = canvasGo.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            var backdrop = NewPanel("Backdrop", canvasGo.transform, ColBackdrop);
            Stretch(backdrop.rectTransform);
            _root = backdrop.gameObject;

            var window = NewPanel("Window", backdrop.transform, ColWindow);
            var wrt = window.rectTransform;
            wrt.anchorMin = wrt.anchorMax = new Vector2(0.5f, 0.5f);
            wrt.pivot = new Vector2(0.5f, 0.5f);
            wrt.sizeDelta = windowSize;

            var vroot = wrt.gameObject.AddComponent<VerticalLayoutGroup>();
            vroot.childForceExpandWidth = false;
            vroot.childForceExpandHeight = false;
            vroot.childControlWidth = true;
            vroot.childControlHeight = true;

            BuildTitleBar(wrt);
            BuildBody(wrt);
            BuildFooter(wrt);
            RebuildTabs();
        }

        private void BuildTitleBar(Transform parent)
        {
            var bar = NewPanel("TitleBar", parent, ColHeader);
            Layout(bar, -1, 46);
            var h = bar.gameObject.AddComponent<HorizontalLayoutGroup>();
            h.padding = new RectOffset(16, 10, 0, 0);
            h.spacing = 8f;
            h.childAlignment = TextAnchor.MiddleLeft;
            h.childForceExpandWidth = false;
            h.childForceExpandHeight = true;
            h.childControlWidth = true;
            h.childControlHeight = true;

            var title = NewText("Title", bar.transform, "DRONE SETTINGS", 20f, ColText);
            Layout(title, -1, -1, 1f);

            var hint = NewText("Hint", bar.transform, "[" + toggleKey + "] close", 13f, ColDim,
                               TextAlignmentOptions.MidlineRight);
            Layout(hint, 130, -1);

            MakeButton(bar.transform, "X", 34, 30, Close, new Color(0.45f, 0.18f, 0.18f, 1f));
        }

        private void BuildBody(Transform parent)
        {
            var body = NewRect("Body", parent);
            Layout(body, -1, windowSize.y - 46f - 54f);
            var h = body.gameObject.AddComponent<HorizontalLayoutGroup>();
            h.spacing = 0f;
            h.childForceExpandWidth = false;
            h.childForceExpandHeight = true;
            h.childControlWidth = true;
            h.childControlHeight = true;

            var side = NewPanel("Tabs", body, ColSide);
            Layout(side, 50, -1);
            var v = side.gameObject.AddComponent<VerticalLayoutGroup>();
            v.padding = new RectOffset(8, 8, 10, 10);
            v.spacing = 4f;
            v.childAlignment = TextAnchor.UpperCenter;
            v.childForceExpandWidth = true;
            v.childForceExpandHeight = false;
            v.childControlWidth = true;
            v.childControlHeight = true;
            _tabsPanel = side.rectTransform;

            var right = NewRect("Panel", body);
            Layout(right, -1, -1, 1f);
            _scroll = MakeScrollView(right, out _content);
            Stretch(_scroll.GetComponent<RectTransform>());
        }

        private void BuildFooter(Transform parent)
        {
            var bar = NewPanel("Footer", parent, ColHeader);
            Layout(bar, -1, 54);
            var h = bar.gameObject.AddComponent<HorizontalLayoutGroup>();
            h.padding = new RectOffset(14, 14, 10, 10);
            h.spacing = 8f;
            h.childAlignment = TextAnchor.MiddleLeft;
            h.childForceExpandWidth = false;
            h.childForceExpandHeight = true;
            h.childControlWidth = true;
            h.childControlHeight = true;

            var presetLabel = NewText("PresetLabel", bar.transform, "Preset", 14f, ColDim);
            Layout(presetLabel, 52, -1);

            _presetField = MakeInput(bar.transform, "default", 150, 30, null, numeric: false);

            MakeButton(bar.transform, "Save", 76, 30, () =>
            {
                DroneSettings.SavePreset(PresetName);
                Status("Saved '" + PresetName + "'");
            }, ColAccent * 0.7f);

            MakeButton(bar.transform, "Load", 76, 30, () =>
            {
                if (DroneSettings.LoadPreset(PresetName)) { RefreshAll(); Status("Loaded '" + PresetName + "'"); }
                else Status("No preset '" + PresetName + "'");
            });

            MakeButton(bar.transform, "Defaults", 92, 30, () =>
            {
                DroneSettings.ResetDefaults(_activeCategory);
                RefreshAll();
                Status("'" + _activeCategory + "' reset");
            });

            _statusText = NewText("Status", bar.transform, "", 13f, ColDim, TextAlignmentOptions.MidlineRight);
            Layout(_statusText, -1, -1, 1f);

            MakeButton(bar.transform, "Close", 82, 30, Close);
        }

        private string PresetName =>
            _presetField != null && !string.IsNullOrWhiteSpace(_presetField.text)
                ? _presetField.text.Trim() : "default";

        private void Status(string msg)
        {
            if (_statusText != null) _statusText.text = msg;
        }

        // ------------------------------------------------------------------ tabs
        private void RebuildTabs()
        {
            if (_tabsPanel == null) return;
            foreach (Transform child in _tabsPanel) Destroy(child.gameObject);
            _tabButtons.Clear();

            var cats = DroneSettings.Categories();
            if (cats.Count == 0)
            {
                NewText("Empty", _tabsPanel, "no settings\nregistered", 13f, ColDim, TextAlignmentOptions.Center);
                return;
            }
            if (string.IsNullOrEmpty(_activeCategory) || !cats.Contains(_activeCategory))
                _activeCategory = cats[0];

            foreach (var cat in cats)
            {
                string c = cat;
                var btn = MakeButton(_tabsPanel, c, -1, 32, () => ShowCategory(c),
                                     c == _activeCategory ? ColAccent * 0.75f : ColField);
                Layout(btn, -1, 32, 1f);
                var lbl = btn.GetComponentInChildren<TextMeshProUGUI>();
                if (lbl != null) lbl.alignment = TextAlignmentOptions.MidlineLeft;
                _tabButtons[c] = btn;
            }
            ShowCategory(_activeCategory);
        }

        private void HighlightTab()
        {
            foreach (var kv in _tabButtons)
            {
                var img = kv.Value.targetGraphic as Image;
                if (img != null) img.color = kv.Key == _activeCategory ? ColAccent * 0.75f : ColField;
            }
        }

        // ------------------------------------------------------------------ rows
        public void ShowCategory(string category)
        {
            if (_content == null) return;
            _activeCategory = category;
            HighlightTab();

            foreach (Transform child in _content) Destroy(child.gameObject);
            _refreshers.Clear();
            _readouts.Clear();

            foreach (var e in DroneSettings.InCategory(category)) BuildRow(e);

            Canvas.ForceUpdateCanvases();
            if (_scroll != null) _scroll.verticalNormalizedPosition = 1f;
        }

        private void BuildRow(SettingEntry e)
        {
            switch (e.kind)
            {
                case SettingKind.Header: BuildHeaderRow(e); break;
                case SettingKind.Button: BuildButtonRow(e); break;
                case SettingKind.Readout: BuildReadoutRow(e); break;
                case SettingKind.Bool: BuildBoolRow(e); break;
                case SettingKind.Vector3: BuildVector3Row(e); break;
                case SettingKind.Enum: BuildEnumRow(e); break;
                case SettingKind.Text: BuildTextRow(e); break;
                default: BuildNumberRow(e); break;
            }
        }

        private void BuildHeaderRow(SettingEntry e)
        {
            var row = MakeRow(_content, 30f);
            var t = NewText("Header", row, e.label.ToUpperInvariant(), 14f, ColAccent);
            t.fontStyle = FontStyles.Bold;
            Layout(t, -1, 26, 1f);
        }

        private void BuildButtonRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            var spacer = NewText("Spacer", row, "", 14f, ColDim);
            Layout(spacer, labelWidth, 28);
            MakeButton(row, e.label, 190, 28, () => { e.click?.Invoke(); RefreshAll(); }, ColAccent * 0.7f);
        }

        private void BuildReadoutRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            var label = NewText("Label", row, e.label, 15f, ColDim);
            Layout(label, labelWidth, 26);

            string initial = "";
            try { initial = e.readout != null ? e.readout() : ""; } catch { }
            var value = NewText("Value", row, initial, 15f, ColWarn);
            Layout(value, -1, 26, 1f);

            _readouts.Add(() =>
            {
                if (value != null && e.readout != null) value.text = e.readout();
            });
        }

        private TextMeshProUGUI RowLabel(RectTransform row, SettingEntry e)
        {
            var t = NewText("Label", row, e.label, 15f, ColText);
            Layout(t, labelWidth, 26);
            return t;
        }

        private void BuildNumberRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            RowLabel(row, e);

            TMP_InputField input = null;
            Slider slider = null;
            bool guard = false;

            input = MakeInput(row, FormatNum(DroneSettings.GetValue(e)), fieldWidth, 26, s =>
            {
                if (guard) return;
                if (!float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v)) return;
                if (e.HasSlider) v = Mathf.Clamp(v, e.min, e.max);
                DroneSettings.SetValue(e, e.kind == SettingKind.Int ? (object)Mathf.RoundToInt(v) : v);
                guard = true;
                if (slider != null) slider.SetValueWithoutNotify(v);
                input.SetTextWithoutNotify(FormatNum(DroneSettings.GetValue(e)));
                guard = false;
            });

            if (e.HasSlider)
            {
                slider = MakeSlider(row, ToFloat(DroneSettings.GetValue(e)), e.min, e.max, v =>
                {
                    if (guard) return;
                    if (e.kind == SettingKind.Int) v = Mathf.Round(v);
                    DroneSettings.SetValue(e, e.kind == SettingKind.Int ? (object)(int)v : v);
                    guard = true;
                    input.SetTextWithoutNotify(FormatNum(DroneSettings.GetValue(e)));
                    guard = false;
                });
                if (e.kind == SettingKind.Int) slider.wholeNumbers = true;
            }

            _refreshers.Add(() =>
            {
                guard = true;
                object val = DroneSettings.GetValue(e);
                input.SetTextWithoutNotify(FormatNum(val));
                slider?.SetValueWithoutNotify(ToFloat(val));
                guard = false;
            });
        }

        private void BuildVector3Row(SettingEntry e)
        {
            var row = MakeRow(_content);
            RowLabel(row, e);

            var inputs = new TMP_InputField[3];
            Vector3 v0 = (Vector3)(DroneSettings.GetValue(e) ?? Vector3.zero);

            for (int i = 0; i < 3; i++)
            {
                int idx = i;
                string sub = e.subLabels != null && i < e.subLabels.Length ? e.subLabels[i] : "";
                if (!string.IsNullOrEmpty(sub))
                {
                    var l = NewText("Sub", row, sub, 13f, ColDim, TextAlignmentOptions.MidlineRight);
                    Layout(l, 16, 26);
                }
                inputs[i] = MakeInput(row, FormatNum(v0[idx]), fieldWidth, 26, s =>
                {
                    if (!float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float f)) return;
                    Vector3 cur = (Vector3)(DroneSettings.GetValue(e) ?? Vector3.zero);
                    cur[idx] = f;
                    DroneSettings.SetValue(e, cur);
                });
            }

            _refreshers.Add(() =>
            {
                Vector3 cur = (Vector3)(DroneSettings.GetValue(e) ?? Vector3.zero);
                for (int i = 0; i < 3; i++) inputs[i].SetTextWithoutNotify(FormatNum(cur[i]));
            });
        }

        private void BuildBoolRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            RowLabel(row, e);
            var toggle = MakeToggle(row, (bool)(DroneSettings.GetValue(e) ?? false),
                                    v => DroneSettings.SetValue(e, v));
            _refreshers.Add(() => toggle.SetIsOnWithoutNotify((bool)(DroneSettings.GetValue(e) ?? false)));
        }

        private void BuildEnumRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            RowLabel(row, e);

            var names = System.Enum.GetNames(e.valueType);
            var values = System.Enum.GetValues(e.valueType);

            int IndexOfCurrent()
            {
                object cur = DroneSettings.GetValue(e);
                for (int i = 0; i < values.Length; i++)
                    if (Equals(values.GetValue(i), cur)) return i;
                return 0;
            }

            TextMeshProUGUI label = null;
            label = MakeCycler(row, names[IndexOfCurrent()], 250f, step =>
            {
                int i = (IndexOfCurrent() + step + names.Length) % names.Length;
                DroneSettings.SetValue(e, values.GetValue(i));
                label.text = names[i];
            });

            _refreshers.Add(() => label.text = names[IndexOfCurrent()]);
        }

        private void BuildTextRow(SettingEntry e)
        {
            var row = MakeRow(_content);
            RowLabel(row, e);
            var input = MakeInput(row, DroneSettings.GetValue(e) as string ?? "", 260, 26,
                                  s => DroneSettings.SetValue(e, s), numeric: false);
            _refreshers.Add(() => input.SetTextWithoutNotify(DroneSettings.GetValue(e) as string ?? ""));
        }

        // ------------------------------------------------------------------ helpers
        /// <summary>Pull fresh values from the game into every visible widget.</summary>
        public void RefreshAll()
        {
            for (int i = 0; i < _refreshers.Count; i++)
            {
                try { _refreshers[i]?.Invoke(); } catch { }
            }
            for (int i = 0; i < _readouts.Count; i++)
            {
                try { _readouts[i]?.Invoke(); } catch { }
            }
        }

        private static float ToFloat(object o)
        {
            try { return Convert.ToSingle(o, CultureInfo.InvariantCulture); } catch { return 0f; }
        }

        private static string FormatNum(object o)
        {
            if (o is int i) return i.ToString(CultureInfo.InvariantCulture);
            float f = ToFloat(o);
            if (Mathf.Approximately(f, Mathf.Round(f)) && Mathf.Abs(f) < 1e5f)
                return f.ToString("0.###", CultureInfo.InvariantCulture);
            return f.ToString("0.####", CultureInfo.InvariantCulture);
        }
    }
}
