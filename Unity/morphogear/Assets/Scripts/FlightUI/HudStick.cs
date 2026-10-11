using UnityEngine;
using UnityEngine.EventSystems;

namespace FlightControlUI
{
    // Per-pointer capture permits two simultaneous touches without one stealing the other.
    public sealed class HudStick : MonoBehaviour, IPointerDownHandler, IDragHandler, IPointerUpHandler, IInitializePotentialDragHandler
    {
        public RectTransform Knob;
        [Range(0, 0.9f)] public float Deadzone = 0.12f;
        public Vector2 Value { get; private set; }
        public bool CircularHeading;
        public bool VerticalOnly;
        public float HeadingRadians { get; private set; }
        public void SetHeading(float radians)
        {
            HeadingRadians = radians;
            if (CircularHeading && Knob != null)
            {
                var r = (RectTransform)transform;
                float radius = Mathf.Max(1, Mathf.Min(r.rect.width, r.rect.height) * 0.5f - Knob.rect.width * 0.5f - 8);
                Knob.anchoredPosition = new Vector2(-Mathf.Sin(radians), Mathf.Cos(radians)) * radius;
            }
        }
        bool inputEnabled;
        int? pointer;
        public bool InputEnabled
        {
            get => inputEnabled;
            set { inputEnabled = value; if (!value) ResetInput(); }
        }
        public void OnInitializePotentialDrag(PointerEventData e) { e.useDragThreshold = false; }
        public void OnPointerDown(PointerEventData e)
        {
            if (!inputEnabled || pointer.HasValue || e.button != PointerEventData.InputButton.Left) return;
            pointer = e.pointerId; OnDrag(e);
        }
        public void OnDrag(PointerEventData e)
        {
            if (!inputEnabled || pointer != e.pointerId) return;
            var r = (RectTransform)transform;
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(r, e.position, e.pressEventCamera, out var local)) return;
            float radius = Mathf.Min(r.rect.width, r.rect.height) * 0.5f - Knob.rect.width * 0.5f - 8;
            Vector2 raw = Vector2.ClampMagnitude((local - r.rect.center) / Mathf.Max(1, radius), 1);
            if (VerticalOnly) raw.x = 0;
            float magnitude = raw.magnitude;
            if (CircularHeading)
            {
                Value = magnitude <= Deadzone ? Vector2.zero : raw.normalized;
                if (magnitude > Deadzone) SetHeading(Mathf.Atan2(-raw.x, raw.y));
                return;
            }
            Value = magnitude <= Deadzone ? Vector2.zero : raw.normalized * ((magnitude - Deadzone) / (1 - Deadzone));
            Knob.anchoredPosition = raw * radius;
        }
        public void OnPointerUp(PointerEventData e) { if (pointer == e.pointerId) ResetInput(); }
        public void ResetInput()
        {
            pointer = null; Value = Vector2.zero;
            if (CircularHeading) SetHeading(HeadingRadians);
            else if (Knob != null) Knob.anchoredPosition = Vector2.zero;
        }
        void OnDisable() => ResetInput();
        void OnApplicationFocus(bool focus) { if (!focus) ResetInput(); }
        void OnApplicationPause(bool paused) { if (paused) ResetInput(); }
    }
}
