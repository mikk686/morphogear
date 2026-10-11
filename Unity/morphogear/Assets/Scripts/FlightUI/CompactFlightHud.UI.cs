using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem.UI;
#endif

namespace FlightControlUI
{
    public sealed partial class CompactFlightHud
    {
        GameObject root, sticks, boundsPanel, detailsPanel;
        HudStick move, lift, heading;
        Text moveTitle, liftTitle, velocityModeLabel;
        Texture2D circleTexture;
        Sprite circleSprite;
        Text statusLine, resultLine, noticeLine, detailText, manualLabel, boundsLabel;
        readonly InputField[] fields = new InputField[7];
        readonly Color background = new Color(0.035f, 0.05f, 0.07f, 0.88f);
        readonly Color foreground = new Color(0.85f, 0.92f, 0.95f);
        readonly Color accent = new Color(0.22f, 0.88f, 0.78f);

        void BuildUI()
        {
            if (font == null) font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            root = new GameObject("Compact Flight HUD [generated]", typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            root.GetComponent<Canvas>().renderMode = RenderMode.ScreenSpaceOverlay;
            root.GetComponent<Canvas>().sortingOrder = 100;
            var scale = root.GetComponent<CanvasScaler>(); scale.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scale.referenceResolution = new Vector2(1400, 900); scale.screenMatchMode = CanvasScaler.ScreenMatchMode.Expand;
            if (FindFirstObjectByType<EventSystem>() == null)
            {
                var e = new GameObject("Flight UI EventSystem", typeof(EventSystem));
#if ENABLE_INPUT_SYSTEM
                e.AddComponent<InputSystemUIInputModule>().AssignDefaultActions();
#else
                e.AddComponent<StandaloneInputModule>();
#endif
            }
            // A single 112-pixel toolbar; no central overlay, crosshair or telemetry cards.
            var top = Panel(root.transform, "Toolbar", 200, 60, 1280, 112);
            statusLine = Label(top, "Waiting for status...", 12, 7, 1345, 22, 15);
            Button(top, "ARM", 12, 34, 78, () => Arm(true));
            Button(top, "DISARM", 96, 34, 88, () => Arm(false));
            Button(top, "STABILIZE", 200, 34, 106, () => SetMode("STABILIZE"));
            Button(top, "POSHOLD", 312, 34, 102, () => SetMode("POSHOLD"));
            Button(top, "GUIDED", 420, 34, 96, () => SetMode("GUIDED"));
            Button(top, "TAKEOFF", 534, 34, 98, Takeoff);
            Button(top, "LAND", 638, 34, 78, Land);
            Button(top, "SET POSITION", 734, 34, 134, SendTarget);
            manualLabel = Button(top, "JOYSTICKS OFF", 882, 34, 154, () => SetManualEnabled(!ManualEnabled)).GetComponentInChildren<Text>();
            Button(top, "BOUNDS", 1052, 34, 110, () => boundsPanel.SetActive(!boundsPanel.activeSelf));
            Button(top, "DETAILS", 1168, 34, 108, () => detailsPanel.SetActive(!detailsPanel.activeSelf));
            resultLine = Label(top, "", 12, 77, 1345, 26, 13);

            boundsPanel = Panel(root.transform, "Bounds editor", 12, 134, 370, 282).gameObject;
            Label(boundsPanel.transform, "BOUNDING BOX / ENU METRES", 14, 10, 342, 24, 16);
            Label(boundsPanel.transform, "", 14, 40, 60, 22, 13);
            Label(boundsPanel.transform, "East X", 82, 40, 86, 22, 13);
            Label(boundsPanel.transform, "North Y", 176, 40, 86, 22, 13);
            Label(boundsPanel.transform, "Up Z", 270, 40, 86, 22, 13);
            string[] defaults = { "-2", "-2", "-0.2", "2", "2", "2.5", "0.2" };
            for (int row = 0; row < 2; row++)
            {
                Label(boundsPanel.transform, row == 0 ? "Min" : "Max", 14, 72 + row * 42, 60, 28, 14);
                for (int axis = 0; axis < 3; axis++)
                {
                    int i = row * 3 + axis;
                    fields[i] = Input(boundsPanel.transform, 82 + axis * 94, 68 + row * 42, 84, defaults[i]);
                }
            }
            Label(boundsPanel.transform, "Margin", 14, 162, 65, 28, 14);
            fields[6] = Input(boundsPanel.transform, 82, 155, 84, defaults[6]);
            Button(boundsPanel.transform, "LOAD SERVER", 14, 198, 150, LoadBounds);
            Button(boundsPanel.transform, "APPLY", 177, 198, 178, ApplyBounds);
            boundsLabel = Label(boundsPanel.transform, "", 14, 244, 342, 30, 12);
            boundsPanel.SetActive(false);

            detailsPanel = Panel(root.transform, "Details", 394, 134, 660, 270).gameObject;
            detailText = Label(detailsPanel.transform, "", 14, 12, 632, 250, 14);
            detailsPanel.SetActive(false);
            sticks = new GameObject("Optional joysticks", typeof(RectTransform)); sticks.transform.SetParent(root.transform, false);
            var sr = sticks.GetComponent<RectTransform>(); sr.anchorMin = Vector2.zero; sr.anchorMax = Vector2.one; sr.offsetMin = sr.offsetMax = Vector2.zero;
            move = Stick(sticks.transform, "EAST / NORTH", false);
            lift = Stick(sticks.transform, "LIFT / YAW", true);
            moveTitle = move.GetComponentInChildren<Text>();
            liftTitle = lift.GetComponentInChildren<Text>();
            heading = Stick(sticks.transform, "GLOBAL YAW", true);
            ((RectTransform)heading.transform).anchoredPosition = new Vector2(-202, 65);
            heading.CircularHeading = true;
            circleTexture = new Texture2D(128, 128, TextureFormat.RGBA32, false);
            var pixels = new Color[128 * 128];
            for (int y = 0; y < 128; y++) for (int x = 0; x < 128; x++)
                pixels[y * 128 + x] = Vector2.Distance(new Vector2(x, y), new Vector2(63.5f, 63.5f)) <= 63 ? Color.white : Color.clear;
            circleTexture.SetPixels(pixels); circleTexture.Apply();
            circleSprite = Sprite.Create(circleTexture, new UnityEngine.Rect(0, 0, 128, 128), new Vector2(0.5f, 0.5f));
            heading.GetComponent<Image>().sprite = circleSprite;
            Label(heading.transform, "E", 70, 28, 20, 20, 12);
            Label(heading.transform, "N", 8, 68, 20, 20, 12);
            Label(heading.transform, "S", 132, 68, 20, 20, 12);
            Label(heading.transform, "W", 70, 131, 20, 20, 12);
            var selector = Button(root.transform, "", 0, 0, 240, CycleVelocityMode);
            var selectorRect = (RectTransform)selector.transform;
            selectorRect.anchorMin = selectorRect.anchorMax = selectorRect.pivot = new Vector2(0.5f, 0);
            selectorRect.anchoredPosition = new Vector2(0, 65);
            velocityModeLabel = selector.GetComponentInChildren<Text>();
            ApplyVelocityMode();
            sticks.SetActive(false);
            var footer = Panel(root.transform, "Notice", 12, 0, 1376, 30);
            footer.anchorMin = footer.anchorMax = footer.pivot = new Vector2(0, 0); footer.anchoredPosition = new Vector2(12, 8);
            noticeLine = Label(footer, message, 10, 4, 1356, 24, 13);
        }
        RectTransform Rect(Transform parent, string name, float x, float y, float w, float h)
        {
            var r = new GameObject(name, typeof(RectTransform)).GetComponent<RectTransform>(); r.SetParent(parent, false);
            r.anchorMin = r.anchorMax = r.pivot = new Vector2(0, 1); r.anchoredPosition = new Vector2(x, -y); r.sizeDelta = new Vector2(w, h); return r;
        }
        RectTransform Panel(Transform parent, string name, float x, float y, float w, float h)
        {
            var r = Rect(parent, name, x, y, w, h); var image = r.gameObject.AddComponent<Image>(); image.color = background; image.raycastTarget = false; return r;
        }
        Text Label(Transform parent, string value, float x, float y, float w, float h, int size)
        {
            var t = Rect(parent, "Text", x, y, w, h).gameObject.AddComponent<Text>();
            t.font = font; t.fontSize = size; t.color = foreground; t.text = value; t.raycastTarget = false; t.supportRichText = false;
            t.verticalOverflow = VerticalWrapMode.Truncate; return t;
        }
        Button Button(Transform parent, string title, float x, float y, float w, UnityAction action)
        {
            var r = Panel(parent, title, x, y, w, 34); var image = r.GetComponent<Image>(); image.color = new Color(0.10f, 0.17f, 0.21f); image.raycastTarget = true;
            var b = r.gameObject.AddComponent<Button>(); b.targetGraphic = image;
            var c = b.colors; c.highlightedColor = new Color(0.6f, 1, 0.9f); c.pressedColor = new Color(0.3f, 0.7f, 0.6f); b.colors = c;
            var nav = b.navigation; nav.mode = Navigation.Mode.None; b.navigation = nav;
            var t = Label(r, title, 0, 0, w, 34, 13); t.alignment = TextAnchor.MiddleCenter;
            b.onClick.AddListener(action); return b;
        }
        InputField Input(Transform parent, float x, float y, float w, string initial)
        {
            var r = Panel(parent, "Numeric input", x, y, w, 32); var image = r.GetComponent<Image>(); image.color = new Color(0.12f, 0.19f, 0.23f); image.raycastTarget = true;
            var t = Label(r, initial, 6, 3, w - 12, 26, 15); t.alignment = TextAnchor.MiddleLeft;
            var input = r.gameObject.AddComponent<InputField>(); input.targetGraphic = image; input.textComponent = t;
            input.contentType = InputField.ContentType.Standard; input.lineType = InputField.LineType.SingleLine; input.text = initial; return input;
        }
        HudStick Stick(Transform parent, string title, bool right)
        {
            var r = Panel(parent, title, 0, 0, 154, 154);
            r.anchorMin = r.anchorMax = r.pivot = new Vector2(right ? 1 : 0, 0);
            r.anchoredPosition = new Vector2(right ? -24 : 24, 65);
            r.GetComponent<Image>().raycastTarget = true;
            Label(r, title, 6, 7, 142, 24, 12).alignment = TextAnchor.MiddleCenter;
            var h = Panel(r, "horizontal", 12, 77, 130, 1); h.GetComponent<Image>().color = new Color(0.3f, 0.5f, 0.5f, 0.6f);
            var v = Panel(r, "vertical", 77, 12, 1, 130); v.GetComponent<Image>().color = new Color(0.3f, 0.5f, 0.5f, 0.6f);
            var knob = Panel(r, "Thumb", 0, 0, 36, 36); knob.anchorMin = knob.anchorMax = knob.pivot = new Vector2(0.5f, 0.5f); knob.anchoredPosition = Vector2.zero;
            knob.GetComponent<Image>().color = accent;
            var stick = r.gameObject.AddComponent<HudStick>(); stick.Knob = knob; stick.Deadzone = 0.12f; return stick;
        }
        static string XYZ(double[] a) => a == null || a.Length < 3 ? "--" : $"{a[0]:F2}, {a[1]:F2}, {a[2]:F2}";
        void Paint()
        {
            var s = Status;
            statusLine.text = s == null ? "FLIGHT  /  waiting for controller" :
                $"FLIGHT  /  {(StatusFresh && s.state_fresh ? "LIVE" : "STALE / OFFLINE")}   |   {(s.armed ? "ARMED" : "DISARMED")}   |   {s.mode}   |   {s.control}   |   ENU {XYZ(s.position)} m   |   {s.operation}";
            var e = s?.last_event;
            resultLine.text = e == null ? "Actions stay clickable. Server rejections appear here." : $"#{e.sequence} {e.command} / {e.result}   {e.detail}";
            noticeLine.text = ManualEnabled && GlobalYaw
                ? $"Global yaw: {heading.HeadingRadians * Mathf.Rad2Deg:F0} deg (E=0, N=90). Drag circle; release retains selection. Lift is separate."
                : message;
            manualLabel.text = !ManualEnabled ? "JOYSTICKS OFF" : HasManualInput ? "TRANSMITTING" : "JOYSTICKS ON";
            boundsLabel.text = s == null ? "No telemetry" : s.bounds_configured ? "Server box configured. Edits need APPLY." : "Not configured. Apply while disarmed.";
            detailText.text = s == null ? "Waiting for status..." :
                $"Movement: {(string.IsNullOrEmpty(s.safety) ? "READY" : s.safety)}\nLast stop: {s.stop_reason}\nFCU message: {s.fcu_text}\nGround confirmed: {s.grounded}\nFrame: {s.frame_id}   Goal: {XYZ(s.goal)}\nBox min: {XYZ(s.box_min)}   max: {XYZ(s.box_max)}\nMargin: {s.boundary_margin:F2} m   Takeoff: {s.takeoff_altitude:F2} m above home\nLimits: {s.max_speed:F2} m/s, {s.max_yaw_rate:F2} rad/s\nLast event: {e?.command} / {e?.result}\n{e?.detail}";
        }
    }
}
