// DroneJoystickHud.cs
// On-screen mouse joysticks + HUD for DroneCommander (ArticulationBody drone).
// Self-building UI - drop the component on any GameObject and press Play.
//
//   LEFT  stick : Y = climb rate (m/s)      X = yaw rate (deg/s)
//   RIGHT stick : Y = forward speed (m/s)   X = strafe speed (m/s)
//
// Uses LocalVelocityControl when available (stick axes are body-frame, no maths needed),
// otherwise falls back to VelocityControl with a yaw rotation.
// Buttons: HOVER / HOLD POS / FREEZE (ArticulationBody.immovable) / RESET.
using System;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;
using TMPro;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace DroneLab
{
    // ======================================================================
    //  One draggable stick
    // ======================================================================
    public class DroneStick : MonoBehaviour, IPointerDownHandler, IDragHandler, IPointerUpHandler
    {
        public RectTransform knob;
        public float radius = 64f;
        public bool selfCenter = true;

        /// <summary>Deflection, each axis in [-1..1]. Y is up.</summary>
        public Vector2 Value { get; private set; }
        public bool Held { get; private set; }

        private RectTransform _rt;

        private void Awake() => _rt = GetComponent<RectTransform>();

        public void OnPointerDown(PointerEventData e) { Held = true; Apply(e); }
        public void OnDrag(PointerEventData e) => Apply(e);

        public void OnPointerUp(PointerEventData e)
        {
            Held = false;
            if (selfCenter) Value = Vector2.zero;
            Place();
        }

        private void Apply(PointerEventData e)
        {
            if (_rt == null) _rt = GetComponent<RectTransform>();
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(
                    _rt, e.position, e.pressEventCamera, out Vector2 local)) return;
            Value = Vector2.ClampMagnitude(local / radius, 1f);
            Place();
        }

        private void Place()
        {
            if (knob != null) knob.anchoredPosition = Value * radius;
        }

        public void Center()
        {
            Value = Vector2.zero;
            Held = false;
            Place();
        }
    }

    // ======================================================================
    //  HUD + command mapping
    // ======================================================================
    [DisallowMultipleComponent]
    public class DroneJoystickHud : MonoBehaviour
    {
        [Header("Targets (auto-found if empty)")]
        public DroneCommander commander;
        [Tooltip("Root ArticulationBody - used for telemetry, FREEZE and RESET.")]
        public ArticulationBody root;
        [Tooltip("Optional: reuse the rig helper for mass / reset / immovable operations.")]
        public DroneRigBindings rig;

        [Header("Command limits")]
        [Tooltip("Max horizontal speed at full right-stick deflection, m/s.")]
        public float maxHorizontalSpeed = 8f;
        [Tooltip("Max climb / descent rate at full left-stick vertical, m/s.")]
        public float maxClimbRate = 3f;
        [Tooltip("Max yaw rate at full left-stick horizontal, deg/s.")]
        public float maxYawRate = 90f;

        [Header("Feel")]
        [Tooltip("Use LocalVelocityControl (body-frame sticks, FPV feel). Off = world-frame VelocityControl.")]
        public bool useLocalVelocity = true;
        [Tooltip("Drive explicit yaw from the left stick. Off = controller auto-yaws along the flight path.")]
        public bool manualYaw = true;
        [Range(1f, 30f)] public float responsiveness = 6f;
        [Range(0f, 0.4f)] public float deadzone = 0.08f;
        [Tooltip("Squares the stick curve for finer control near centre.")]
        public bool expoCurve = true;

        [Header("Extras")]
        public bool keyboardFallback = true;    // WASD + RF + QE
        public bool showHud = true;
        public int sortingOrder = 200;

        // ---- runtime
        private DroneStick _left, _right;
        private TextMeshProUGUI _hud, _modeText;
        private Transform _drone;
        private Vector3 _cmdVel;
        private float _yawTarget;
        private bool _frozen;

        private static readonly Color ColBase = new Color(0.10f, 0.12f, 0.15f, 0.55f);
        private static readonly Color ColKnob = new Color(0.25f, 0.80f, 0.55f, 0.95f);
        private static readonly Color ColText = new Color(0.88f, 0.92f, 0.95f, 1f);
        private static readonly Color ColDim = new Color(0.65f, 0.70f, 0.75f, 1f);
        private static readonly Color ColBtn = new Color(0.14f, 0.17f, 0.21f, 0.92f);
        private static readonly Color ColFreeze = new Color(0.45f, 0.30f, 0.12f, 0.95f);

        // ------------------------------------------------------------------
        private void Start()
        {
            if (commander == null) commander = FindFirstObjectByType<DroneCommander>(FindObjectsInactive.Include);
            if (commander == null)
            {
                Debug.LogError("[DroneJoystickHud] No DroneCommander found.");
                enabled = false;
                return;
            }
            if (rig == null) rig = FindFirstObjectByType<DroneRigBindings>(FindObjectsInactive.Include);
            if (root == null)
            {
                root = rig != null && rig.root != null
                    ? rig.root
                    : commander.GetComponentInChildren<ArticulationBody>(true);
            }

            _drone = root != null ? root.transform : commander.transform;
            _yawTarget = _drone.eulerAngles.y;

            BuildUi();
        }

        private void Update()
        {
            if (commander == null) return;
            float dt = Time.deltaTime;
            if (dt <= 0f) return;                      // paused by the settings menu

            Vector2 l = Shape(_left != null ? _left.Value : Vector2.zero);
            Vector2 r = Shape(_right != null ? _right.Value : Vector2.zero);

            if (keyboardFallback)
            {
                Vector2 kl = KeyLeft(), kr = KeyRight();
                if (kl.sqrMagnitude > 0f) l = kl;
                if (kr.sqrMagnitude > 0f) r = kr;
            }

            // ---- desired velocity from the sticks
            Vector3 planar = new Vector3(r.x, 0f, r.y) * maxHorizontalSpeed;
            planar = Vector3.ClampMagnitude(planar, maxHorizontalSpeed);
            Vector3 desired = new Vector3(planar.x, l.y * maxClimbRate, planar.z);

            if (!useLocalVelocity)
            {
                // world-frame command: rotate the body-frame stick input by the current heading
                Vector3 flat = Quaternion.Euler(0f, _drone.eulerAngles.y, 0f) * new Vector3(desired.x, 0f, desired.z);
                desired = new Vector3(flat.x, desired.y, flat.z);
            }

            float k = 1f - Mathf.Exp(-responsiveness * dt);
            _cmdVel = Vector3.Lerp(_cmdVel, desired, k);
            if (_cmdVel.sqrMagnitude < 1e-4f) _cmdVel = Vector3.zero;

            commander.currentMode = useLocalVelocity
                ? DroneCommander.CommanderMode.LocalVelocityControl
                : DroneCommander.CommanderMode.VelocityControl;
            commander.targetVelocity = _cmdVel;

            // ---- yaw
            if (manualYaw)
            {
                _yawTarget += l.x * maxYawRate * dt;
                _yawTarget = Mathf.Repeat(_yawTarget, 360f);
                commander.useExplicitYaw = true;
                commander.explicitYawAngle = _yawTarget;
            }
            else
            {
                commander.useExplicitYaw = false;
                _yawTarget = _drone.eulerAngles.y;
            }

            UpdateHud();
        }

        // ------------------------------------------------------------------ shaping
        private Vector2 Shape(Vector2 v) => new Vector2(Axis(v.x), Axis(v.y));

        private float Axis(float a)
        {
            if (Mathf.Abs(a) < deadzone) return 0f;
            float s = Mathf.Sign(a);
            float m = (Mathf.Abs(a) - deadzone) / (1f - deadzone);
            if (expoCurve) m *= m;
            return s * Mathf.Clamp01(m);
        }

        private Vector2 KeyLeft()
        {
            float x = 0f, y = 0f;
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.rKey.isPressed) y += 1f;
                if (kb.fKey.isPressed) y -= 1f;
                if (kb.eKey.isPressed) x += 1f;
                if (kb.qKey.isPressed) x -= 1f;
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKey(KeyCode.R)) y += 1f;
            if (Input.GetKey(KeyCode.F)) y -= 1f;
            if (Input.GetKey(KeyCode.E)) x += 1f;
            if (Input.GetKey(KeyCode.Q)) x -= 1f;
#endif
            return new Vector2(x, y);
        }

        private Vector2 KeyRight()
        {
            float x = 0f, y = 0f;
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.wKey.isPressed) y += 1f;
                if (kb.sKey.isPressed) y -= 1f;
                if (kb.dKey.isPressed) x += 1f;
                if (kb.aKey.isPressed) x -= 1f;
            }
#elif ENABLE_LEGACY_INPUT_MANAGER
            if (Input.GetKey(KeyCode.W)) y += 1f;
            if (Input.GetKey(KeyCode.S)) y -= 1f;
            if (Input.GetKey(KeyCode.D)) x += 1f;
            if (Input.GetKey(KeyCode.A)) x -= 1f;
#endif
            return new Vector2(x, y);
        }

        // ------------------------------------------------------------------ actions
        /// <summary>Zero the velocity command and centre both sticks.</summary>
        public void Hover()
        {
            _left?.Center();
            _right?.Center();
            _cmdVel = Vector3.zero;
            if (commander == null) return;
            commander.currentMode = useLocalVelocity
                ? DroneCommander.CommanderMode.LocalVelocityControl
                : DroneCommander.CommanderMode.VelocityControl;
            commander.targetVelocity = Vector3.zero;
        }

        /// <summary>Freeze on the spot with the position loop.</summary>
        public void HoldPosition()
        {
            if (commander == null) return;
            Hover();
            commander.targetPosition = _drone != null ? _drone.position : commander.transform.position;
            commander.currentMode = DroneCommander.CommanderMode.PositionControl;
        }

        /// <summary>Toggle ArticulationBody.immovable - pins the drone in mid-air for PID tuning.</summary>
        public void ToggleFreeze()
        {
            if (root == null) return;
            if (!root.isRoot)
            {
                Debug.LogWarning("[DroneJoystickHud] 'immovable' only applies to the articulation root.");
                return;
            }
            _frozen = !root.immovable;
            root.immovable = _frozen;
            if (!_frozen) root.WakeUp();
        }

        /// <summary>Teleport back to spawn (delegated to the rig helper when present).</summary>
        public void ResetDrone()
        {
            Hover();
            if (rig != null) { rig.ResetPose(); return; }
            if (root != null && root.isRoot) root.TeleportRoot(root.transform.position, Quaternion.identity);
        }

        public void SyncYaw()
        {
            if (_drone != null) _yawTarget = _drone.eulerAngles.y;
        }

        // ------------------------------------------------------------------ hud
        private void UpdateHud()
        {
            if (!showHud || _hud == null) return;

            Vector3 v = root != null ? root.linearVelocity : Vector3.zero;
            float mass = rig != null ? rig.TotalMass() : (root != null ? root.mass : 0f);
            bool pinned = root != null && root.isRoot && root.immovable;

            string mode = pinned
                ? "<color=#E0A44C>IMMOVABLE</color>"
                : (useLocalVelocity ? "<color=#42D68C>LOCAL VEL</color>" : "<color=#42D68C>WORLD VEL</color>");

            _hud.text =
                mode + "   alt " + (_drone != null ? _drone.position.y : 0f).ToString("F1") + " m" +
                "   hdg " + (_drone != null ? _drone.eulerAngles.y : 0f).ToString("F0") + " deg" +
                "   mass " + mass.ToString("F2") + " kg\n" +
                "cmd (" + _cmdVel.x.ToString("F1") + "," + _cmdVel.y.ToString("F1") + "," + _cmdVel.z.ToString("F1") + ")" +
                "   act (" + v.x.ToString("F1") + "," + v.y.ToString("F1") + "," + v.z.ToString("F1") + ")" +
                "   " + v.magnitude.ToString("F1") + " m/s";

            if (_modeText != null)
                _modeText.text = commander != null ? commander.currentMode.ToString() : "";
        }

        // ==================================================================
        //  UI construction
        // ==================================================================
        private void BuildUi()
        {
            EnsureEventSystem();

            var canvasGo = new GameObject("Drone Joystick Canvas",
                typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            canvasGo.transform.SetParent(transform, false);
            var canvas = canvasGo.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = sortingOrder;
            var scaler = canvasGo.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);
            scaler.matchWidthOrHeight = 0.5f;

            _left = MakeStick(canvas.transform, "Left Stick", new Vector2(0f, 0f), new Vector2(190f, 150f));
            _right = MakeStick(canvas.transform, "Right Stick", new Vector2(1f, 0f), new Vector2(-190f, 150f));

            MakeLabel(canvas.transform, "L: throttle / yaw", new Vector2(0f, 0f), new Vector2(190f, 22f), 14f, ColDim);
            MakeLabel(canvas.transform, "R: forward / strafe", new Vector2(1f, 0f), new Vector2(-190f, 22f), 14f, ColDim);

            _hud = MakeLabel(canvas.transform, "", new Vector2(0.5f, 0f), new Vector2(0f, 104f), 16f, ColText);
            _hud.rectTransform.sizeDelta = new Vector2(1000f, 50f);
            _hud.richText = true;

            _modeText = MakeLabel(canvas.transform, "", new Vector2(0.5f, 1f), new Vector2(0f, -28f), 16f, ColDim);
            _modeText.rectTransform.sizeDelta = new Vector2(420f, 26f);

            MakeButton(canvas.transform, "HOVER", new Vector2(0.5f, 0f), new Vector2(-210f, 44f), Hover, ColBtn);
            MakeButton(canvas.transform, "HOLD POS", new Vector2(0.5f, 0f), new Vector2(-70f, 44f), HoldPosition, ColBtn);
            MakeButton(canvas.transform, "FREEZE", new Vector2(0.5f, 0f), new Vector2(70f, 44f), ToggleFreeze, ColFreeze);
            MakeButton(canvas.transform, "RESET", new Vector2(0.5f, 0f), new Vector2(210f, 44f), ResetDrone, ColBtn);
        }

        private DroneStick MakeStick(Transform parent, string name, Vector2 anchor, Vector2 pos)
        {
            const float R = 76f;

            var baseGo = new GameObject(name, typeof(RectTransform), typeof(Image), typeof(DroneStick));
            baseGo.transform.SetParent(parent, false);
            var brt = baseGo.GetComponent<RectTransform>();
            brt.anchorMin = brt.anchorMax = anchor;
            brt.pivot = new Vector2(0.5f, 0.5f);
            brt.anchoredPosition = pos;
            brt.sizeDelta = new Vector2(R * 2f, R * 2f);
            baseGo.GetComponent<Image>().color = ColBase;

            MakeBar(brt, new Vector2(R * 2f - 20f, 1.5f));
            MakeBar(brt, new Vector2(1.5f, R * 2f - 20f));

            var knobGo = new GameObject("Knob", typeof(RectTransform), typeof(Image));
            knobGo.transform.SetParent(baseGo.transform, false);
            var krt = knobGo.GetComponent<RectTransform>();
            krt.anchorMin = krt.anchorMax = new Vector2(0.5f, 0.5f);
            krt.pivot = new Vector2(0.5f, 0.5f);
            krt.sizeDelta = new Vector2(46f, 46f);
            var kimg = knobGo.GetComponent<Image>();
            kimg.color = ColKnob;
            kimg.raycastTarget = false;

            var stick = baseGo.GetComponent<DroneStick>();
            stick.knob = krt;
            stick.radius = R - 12f;
            stick.selfCenter = true;
            return stick;
        }

        private void MakeBar(Transform parent, Vector2 size)
        {
            var go = new GameObject("Guide", typeof(RectTransform), typeof(Image));
            go.transform.SetParent(parent, false);
            var rt = go.GetComponent<RectTransform>();
            rt.anchorMin = rt.anchorMax = new Vector2(0.5f, 0.5f);
            rt.sizeDelta = size;
            var img = go.GetComponent<Image>();
            img.color = new Color(1f, 1f, 1f, 0.12f);
            img.raycastTarget = false;
        }

        private TextMeshProUGUI MakeLabel(Transform parent, string text, Vector2 anchor, Vector2 pos,
                                          float size, Color color)
        {
            var go = new GameObject("Label", typeof(RectTransform), typeof(TextMeshProUGUI));
            go.transform.SetParent(parent, false);
            var rt = go.GetComponent<RectTransform>();
            rt.anchorMin = rt.anchorMax = anchor;
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = pos;
            rt.sizeDelta = new Vector2(280f, 24f);

            var t = go.GetComponent<TextMeshProUGUI>();
            var font = TMP_Settings.defaultFontAsset;
            if (font != null) t.font = font;
            else Debug.LogWarning("[DroneJoystickHud] No TMP default font. Import TMP Essential Resources.");
            t.text = text;
            t.fontSize = size;
            t.color = color;
            t.alignment = TextAlignmentOptions.Center;
            t.raycastTarget = false;
            return t;
        }

        private void MakeButton(Transform parent, string label, Vector2 anchor, Vector2 pos,
                                Action onClick, Color color)
        {
            var go = new GameObject(label, typeof(RectTransform), typeof(Image), typeof(Button));
            go.transform.SetParent(parent, false);
            var rt = go.GetComponent<RectTransform>();
            rt.anchorMin = rt.anchorMax = anchor;
            rt.pivot = new Vector2(0.5f, 0.5f);
            rt.anchoredPosition = pos;
            rt.sizeDelta = new Vector2(130f, 34f);
            go.GetComponent<Image>().color = color;

            var t = MakeLabel(go.transform, label, new Vector2(0.5f, 0.5f), Vector2.zero, 14f, ColText);
            t.rectTransform.sizeDelta = new Vector2(130f, 34f);

            go.GetComponent<Button>().onClick.AddListener(() => onClick?.Invoke());
        }

        private static void EnsureEventSystem()
        {
            if (FindFirstObjectByType<EventSystem>(FindObjectsInactive.Include) != null) return;
            var es = new GameObject("EventSystem", typeof(EventSystem));
#if ENABLE_INPUT_SYSTEM
            es.AddComponent<UnityEngine.InputSystem.UI.InputSystemUIInputModule>();
#else
            es.AddComponent<StandaloneInputModule>();
#endif
        }
    }
}
