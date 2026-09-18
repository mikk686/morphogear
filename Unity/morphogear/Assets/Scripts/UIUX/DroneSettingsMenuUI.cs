// DroneSettingsMenuUI.cs
// TextMeshPro widget factory for DroneSettingsMenu (partial class - UI plumbing only).
using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using TMPro;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem.UI;
#endif

namespace DroneLab
{
    public partial class DroneSettingsMenu
    {
        // ------------------------------------------------------------------ theme
        private static readonly Color ColBackdrop = new Color(0f, 0f, 0f, 0.55f);
        private static readonly Color ColWindow = new Color(0.09f, 0.10f, 0.12f, 0.97f);
        private static readonly Color ColHeader = new Color(0.13f, 0.15f, 0.18f, 1f);
        private static readonly Color ColSide = new Color(0.11f, 0.12f, 0.15f, 1f);
        private static readonly Color ColField = new Color(0.17f, 0.19f, 0.23f, 1f);
        private static readonly Color ColAccent = new Color(0.20f, 0.72f, 0.45f, 1f);
        private static readonly Color ColWarn = new Color(0.85f, 0.55f, 0.20f, 1f);
        private static readonly Color ColText = new Color(0.90f, 0.93f, 0.95f, 1f);
        private static readonly Color ColDim = new Color(0.62f, 0.66f, 0.70f, 1f);

        private TMP_FontAsset _font;

        private TMP_FontAsset Font
        {
            get
            {
                if (_font != null) return _font;
                _font = TMP_Settings.defaultFontAsset;
                if (_font == null) _font = Resources.Load<TMP_FontAsset>("Fonts & Materials/LiberationSans SDF");
                if (_font == null)
                    Debug.LogError("[DroneSettingsMenu] No TMP font asset. " +
                                   "Run Window > TextMeshPro > Import TMP Essential Resources.");
                return _font;
            }
        }

        // ------------------------------------------------------------------ primitives
        private static RectTransform NewRect(string name, Transform parent)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            return go.GetComponent<RectTransform>();
        }

        private static Image NewPanel(string name, Transform parent, Color color)
        {
            var rt = NewRect(name, parent);
            var img = rt.gameObject.AddComponent<Image>();
            img.color = color;
            return img;
        }

        private static void Stretch(RectTransform rt, float l = 0, float b = 0, float r = 0, float t = 0)
        {
            rt.anchorMin = Vector2.zero;
            rt.anchorMax = Vector2.one;
            rt.offsetMin = new Vector2(l, b);
            rt.offsetMax = new Vector2(-r, -t);
        }

        private TextMeshProUGUI NewText(string name, Transform parent, string text, float size,
                                        Color color, TextAlignmentOptions align = TextAlignmentOptions.MidlineLeft)
        {
            var rt = NewRect(name, parent);
            var t = rt.gameObject.AddComponent<TextMeshProUGUI>();
            if (Font != null) t.font = Font;
            t.text = text;
            t.fontSize = size;
            t.color = color;
            t.alignment = align;
            t.raycastTarget = false;
            t.overflowMode = TextOverflowModes.Ellipsis;
            return t;
        }

        private static LayoutElement Layout(Component c, float width = -1, float height = -1, float flexW = -1)
        {
            var le = c.gameObject.GetComponent<LayoutElement>() ?? c.gameObject.AddComponent<LayoutElement>();
            if (width >= 0) le.preferredWidth = le.minWidth = width;
            if (height >= 0) le.preferredHeight = le.minHeight = height;
            if (flexW >= 0) le.flexibleWidth = flexW;
            return le;
        }

        // ------------------------------------------------------------------ widgets
        private TMP_InputField MakeInput(Transform parent, string value, float width, float height,
                                         Action<string> onSubmit, bool numeric = true)
        {
            var bg = NewPanel("Input", parent, ColField);
            Layout(bg, width, height);

            var viewport = NewRect("Text Area", bg.transform);
            Stretch(viewport, 8, 3, 8, 3);
            viewport.gameObject.AddComponent<RectMask2D>();

            var text = NewText("Text", viewport, value, 15f, ColText);
            Stretch(text.rectTransform);
            text.richText = false;

            var input = bg.gameObject.AddComponent<TMP_InputField>();
            input.textViewport = viewport;
            input.textComponent = text;
            if (Font != null) input.fontAsset = Font;
            input.pointSize = 15f;
            input.targetGraphic = bg;
            input.selectionColor = new Color(0.2f, 0.6f, 0.9f, 0.5f);
            input.caretColor = ColText;
            input.customCaretColor = true;
            input.contentType = numeric
                ? TMP_InputField.ContentType.DecimalNumber
                : TMP_InputField.ContentType.Standard;
            input.lineType = TMP_InputField.LineType.SingleLine;
            input.restoreOriginalTextOnEscape = true;
            input.text = value;

            input.onEndEdit.AddListener(s => onSubmit?.Invoke(s));
            input.onSubmit.AddListener(s => onSubmit?.Invoke(s));
            return input;
        }

        private Toggle MakeToggle(Transform parent, bool value, Action<bool> onChange)
        {
            var bg = NewPanel("Toggle", parent, ColField);
            Layout(bg, 26, 26);

            var check = NewPanel("Check", bg.transform, ColAccent);
            Stretch(check.rectTransform, 5, 5, 5, 5);

            var toggle = bg.gameObject.AddComponent<Toggle>();
            toggle.targetGraphic = bg;
            toggle.graphic = check;
            toggle.isOn = value;
            toggle.onValueChanged.AddListener(v => onChange?.Invoke(v));
            return toggle;
        }

        private Slider MakeSlider(Transform parent, float value, float min, float max, Action<float> onChange)
        {
            var root = NewRect("Slider", parent);
            Layout(root, -1, 20, 1f);
            var slider = root.gameObject.AddComponent<Slider>();

            var bg = NewPanel("Background", root, ColField);
            bg.rectTransform.anchorMin = new Vector2(0f, 0.35f);
            bg.rectTransform.anchorMax = new Vector2(1f, 0.65f);
            bg.rectTransform.offsetMin = bg.rectTransform.offsetMax = Vector2.zero;

            var fillArea = NewRect("Fill Area", root);
            fillArea.anchorMin = new Vector2(0f, 0.35f);
            fillArea.anchorMax = new Vector2(1f, 0.65f);
            fillArea.offsetMin = fillArea.offsetMax = Vector2.zero;
            var fill = NewPanel("Fill", fillArea, ColAccent);
            fill.rectTransform.anchorMin = Vector2.zero;
            fill.rectTransform.anchorMax = Vector2.one;
            fill.rectTransform.offsetMin = fill.rectTransform.offsetMax = Vector2.zero;

            var handleArea = NewRect("Handle Slide Area", root);
            Stretch(handleArea, 8, 0, 8, 0);
            var handle = NewPanel("Handle", handleArea, ColText);
            handle.rectTransform.sizeDelta = new Vector2(14f, 18f);

            slider.fillRect = fill.rectTransform;
            slider.handleRect = handle.rectTransform;
            slider.targetGraphic = handle;
            slider.direction = Slider.Direction.LeftToRight;
            slider.minValue = min;
            slider.maxValue = max;
            slider.SetValueWithoutNotify(Mathf.Clamp(value, min, max));
            slider.onValueChanged.AddListener(v => onChange?.Invoke(v));
            return slider;
        }

        private Button MakeButton(Transform parent, string label, float width, float height,
                                  Action onClick, Color? color = null)
        {
            var bg = NewPanel("Button", parent, color ?? ColField);
            Layout(bg, width, height);

            var t = NewText("Label", bg.transform, label, 15f, ColText, TextAlignmentOptions.Center);
            Stretch(t.rectTransform, 4, 0, 4, 0);

            var btn = bg.gameObject.AddComponent<Button>();
            btn.targetGraphic = bg;
            var colors = btn.colors;
            colors.normalColor = Color.white;
            colors.highlightedColor = new Color(1.25f, 1.25f, 1.25f, 1f);
            colors.pressedColor = new Color(0.8f, 0.8f, 0.8f, 1f);
            colors.fadeDuration = 0.06f;
            btn.colors = colors;
            btn.onClick.AddListener(() => onClick?.Invoke());
            return btn;
        }

        /// <summary>"&lt; Value &gt;" cycler - used for enums, no dropdown template required.</summary>
        private TextMeshProUGUI MakeCycler(Transform parent, string value, float width, Action<int> onStep)
        {
            var row = NewRect("Cycler", parent);
            var h = row.gameObject.AddComponent<HorizontalLayoutGroup>();
            h.spacing = 4f;
            h.childForceExpandWidth = false;
            h.childForceExpandHeight = true;
            h.childControlWidth = true;
            h.childControlHeight = true;
            Layout(row, width, 26);

            MakeButton(row, "<", 26, 26, () => onStep?.Invoke(-1));
            var box = NewPanel("Value", row, ColField);
            Layout(box, width - 60f, 26);
            var t = NewText("Text", box.transform, value, 15f, ColText, TextAlignmentOptions.Center);
            Stretch(t.rectTransform, 4, 0, 4, 0);
            MakeButton(row, ">", 26, 26, () => onStep?.Invoke(1));
            return t;
        }

        // ------------------------------------------------------------------ containers
        private RectTransform MakeRow(Transform parent, float height = 30f, float spacing = 8f)
        {
            var row = NewRect("Row", parent);
            var h = row.gameObject.AddComponent<HorizontalLayoutGroup>();
            h.spacing = spacing;
            h.childAlignment = TextAnchor.MiddleLeft;
            h.childForceExpandWidth = false;
            h.childForceExpandHeight = false;
            h.childControlWidth = true;
            h.childControlHeight = true;
            Layout(row, -1, height);
            return row;
        }

        private ScrollRect MakeScrollView(Transform parent, out RectTransform content)
        {
            var root = NewRect("ScrollView", parent);
            var scroll = root.gameObject.AddComponent<ScrollRect>();

            var viewport = NewPanel("Viewport", root, new Color(0f, 0f, 0f, 0.001f));
            Stretch(viewport.rectTransform);
            viewport.gameObject.AddComponent<RectMask2D>();

            content = NewRect("Content", viewport.transform);
            content.anchorMin = new Vector2(0f, 1f);
            content.anchorMax = new Vector2(1f, 1f);
            content.pivot = new Vector2(0.5f, 1f);
            content.offsetMin = content.offsetMax = Vector2.zero;

            var v = content.gameObject.AddComponent<VerticalLayoutGroup>();
            v.padding = new RectOffset(14, 14, 12, 12);
            v.spacing = 6f;
            v.childAlignment = TextAnchor.UpperLeft;
            v.childForceExpandWidth = true;
            v.childForceExpandHeight = false;
            v.childControlWidth = true;
            v.childControlHeight = true;

            var fitter = content.gameObject.AddComponent<ContentSizeFitter>();
            fitter.verticalFit = ContentSizeFitter.FitMode.PreferredSize;

            scroll.viewport = viewport.rectTransform;
            scroll.content = content;
            scroll.horizontal = false;
            scroll.vertical = true;
            scroll.movementType = ScrollRect.MovementType.Clamped;
            scroll.scrollSensitivity = 28f;
            return scroll;
        }

        private static void EnsureEventSystem()
        {
            if (FindFirstObjectByType<EventSystem>(FindObjectsInactive.Include) != null) return;
            var es = new GameObject("EventSystem", typeof(EventSystem));
#if ENABLE_INPUT_SYSTEM
            es.AddComponent<InputSystemUIInputModule>();
#else
            es.AddComponent<StandaloneInputModule>();
#endif
        }
    }
}
