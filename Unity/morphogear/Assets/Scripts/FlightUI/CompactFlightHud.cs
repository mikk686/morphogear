using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;
using System.Globalization;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace FlightControlUI
{
    public sealed partial class CompactFlightHud : MonoBehaviour
    {
        public Transform target;
        [Tooltip("Rigid map origin. Local X east, Y up, Z north. One unit = one metre.")]
        public Transform rosOrigin;
        public string frameId = "map";
        [Tooltip("Empty for root topics. Example: /drone1")]
        public string commandNamespace = "";
        public string statusTopic = "/state_info";
        public float linearSpeed = 0.5f, yawRate = 0.5f, publishHz = 20;
        public Font font;
        public enum VelocityMode { EnuYawRate, BodyYawRate, EnuGlobalYaw, BodyGlobalYaw }
        public VelocityMode velocityMode = VelocityMode.EnuYawRate;
        bool BodyFrame => velocityMode == VelocityMode.BodyYawRate || velocityMode == VelocityMode.BodyGlobalYaw;
        bool GlobalYaw => velocityMode == VelocityMode.EnuGlobalYaw || velocityMode == VelocityMode.BodyGlobalYaw;
        VelocityMode appliedVelocityMode;
        bool headingInitialized;


        [Tooltip("Send joystick motion to the Unity simulation without requiring FCU status.")]
        public bool simulationFlightSticks = false;///

        bool simulationMotionSent;///
        bool lastMotionBodyFrame;///
        bool lastMotionGlobalYaw;///
        float lastMotionYaw;///



        [Serializable] class MotionCommand
        {
            public string frame_id;
            public float[] values;
            public bool is_body_frame, use_yaw_rate;
        }
        [Serializable] public class ControllerEvent { public long sequence; public string command, result, detail; }
        [Serializable] public class ControllerState
        {
            public bool connected, state_fresh, armed, grounded, bounds_configured, yaw_valid;
            public float yaw_enu;
            public string mode, control, operation, safety, stop_reason, fcu_text, frame_id;
            public double[] position, goal, box_min, box_max;
            public float boundary_margin, takeoff_altitude, max_speed, max_yaw_rate;
            public ControllerEvent last_event;
        }
        [Serializable] class BoundsCommand { public float[] min, max; public float margin; }
        public ControllerState Status { get; private set; }
        public bool ManualEnabled { get; private set; }
        public bool StatusFresh => Status != null && Time.realtimeSinceStartupAsDouble - received < 2;
        //public bool MotionReady => StatusFresh && Status.state_fresh && Status.armed && Status.mode == "GUIDED" &&
        //    string.IsNullOrEmpty(Status.safety) && string.IsNullOrEmpty(Status.operation);

        public bool MotionReady => simulationFlightSticks ||
                    (StatusFresh && Status.state_fresh && Status.armed &&
                     Status.mode == "GUIDED" &&
                     string.IsNullOrEmpty(Status.safety) &&
                     string.IsNullOrEmpty(Status.operation)); ///
        // Other local publishers can yield on this property. ROS itself provides no ownership arbitration.
        public bool HasManualInput => isActiveAndEnabled && focused && ManualEnabled && MotionReady &&
            (move.Value.sqrMagnitude > 0 || lift.Value.sqrMagnitude > 0 ||
             (GlobalYaw && heading.Value.sqrMagnitude > 0));
        ROSConnection ros;
        double received = double.NegativeInfinity, nextSend, nextPaint, confirmDisarmUntil;
        bool focused = true;
        string message = "Ready. ARM and mode buttons do not depend on pose or bounds.";
        string ns;
        string Topic(string name) => ns + "/" + name;

        void Start()
        {
            ns = "/" + commandNamespace.Trim('/');
            if (ns == "/") ns = "";
            BuildUI();
#if ROS2
            ros = ROSConnection.GetOrCreateInstance();
            Register<BoolMsg>("arm_unity"); Register<BoolMsg>("takeoff_unity"); Register<BoolMsg>("land_unity");
            Register<StringMsg>("motion_unity"); Register<StringMsg>("mode_unity"); Register<StringMsg>("bounds_unity");
            Register<PoseStampedMsg>("b3/waypoint"); Register<TwistStampedMsg>("setvel_unity");
            var weak = new WeakReference<CompactFlightHud>(this);
            ros.Subscribe<StringMsg>(statusTopic, msg =>
            {
                if (!weak.TryGetTarget(out var hud) || hud == null || !hud.isActiveAndEnabled) return;
                try
                {
                    var s = JsonUtility.FromJson<ControllerState>(msg.data);
                    if (s == null || s.last_event == null) return;
                    hud.Status = s; hud.received = Time.realtimeSinceStartupAsDouble;
                }
                catch (Exception e) { hud.message = "Invalid state JSON: " + e.Message; }
            });
#else
            message = "Select ROS2 in Robotics / ROS Settings. ROS2 scripting define is required.";
#endif
        }
        void Register<T>(string name) where T : Message
        {
            var old = ros.GetTopic(Topic(name));
            if (old != null && old.RosMessageName != MessageRegistry.GetRosMessageName<T>())
                throw new InvalidOperationException("Topic type mismatch: " + Topic(name));
            if (old == null || !old.IsPublisher) ros.RegisterPublisher<T>(Topic(name), queue_size: 1, latch: false);
        }
        bool Send(string name, Message data)
        {
            if (ros == null || ros.HasConnectionError) { message = "Not sent: ROS transport unavailable."; return false; }
            try { ros.Publish(Topic(name), data); return true; }
            catch (Exception e) { message = e.Message; return false; }
        }
        public void SetManualEnabled(bool value)
        {
            if (!value)
                StopSimulationSticks();///
            ManualEnabled = value;
            move?.ResetInput(); lift?.ResetInput(); heading?.ResetInput();
            headingInitialized = false;
            //if (value && StatusFresh && Status.yaw_valid)
            //{ heading?.SetHeading(Status.yaw_enu); headingInitialized = true; }

            if (value) TryInitializeHeading(); ///
            if (sticks != null) sticks.SetActive(value);
        }
        public void CycleVelocityMode()
        {
            velocityMode = (VelocityMode)(((int)velocityMode + 1) % 4);
            ApplyVelocityMode();
        }
        void ApplyVelocityMode()
        {
            SetManualEnabled(false);
            appliedVelocityMode = velocityMode;
            heading.gameObject.SetActive(GlobalYaw);
            lift.VerticalOnly = GlobalYaw;
            moveTitle.text = BodyFrame ? "FORWARD / LEFT" : "EAST UP / NORTH LEFT";
            liftTitle.text = GlobalYaw ? "LIFT" : "LIFT / YAW RATE";
            velocityModeLabel.text = (BodyFrame ? "BODY" : "ENU") + (GlobalYaw ? " + GLOBAL YAW" : " + YAW RATE");
            message = "Velocity mode changed. Enable joysticks to control.";
        }
        public void Arm(bool value)
        {
            SetManualEnabled(false);
            if (!value && Time.realtimeSinceStartupAsDouble > confirmDisarmUntil)
            {
                confirmDisarmUntil = Time.realtimeSinceStartupAsDouble + 3;
                message = "Click DISARM again within 3 seconds. FCU disarming rules apply.";
                return;
            }
            confirmDisarmUntil = 0;
            if (Send("arm_unity", new BoolMsg(value))) message = (value ? "Arm" : "Disarm") + " requested; check controller/FCU result.";
        }
        public void SetMode(string mode)
        {
            SetManualEnabled(false);
            if (Send("mode_unity", new StringMsg(mode))) message = mode + " requested; wait for reported mode.";
        }
        public void Takeoff() { SetManualEnabled(false); if (Send("takeoff_unity", new BoolMsg(true))) message = "Takeoff requested."; }
        public void Land() { SetManualEnabled(false); if (Send("land_unity", new BoolMsg(true))) message = "LAND requested; verify mode and touchdown."; }
        //Vector3 ToENU(Vector3 world)
        //{
        //    var p = rosOrigin == null ? world : Quaternion.Inverse(rosOrigin.rotation) * (world - rosOrigin.position);
        //    return new Vector3(p.x, p.z, p.y);
        //}
        public void SendTarget()
        {
            if (target == null) { message = "Assign Target in Inspector."; return; }
            SetManualEnabled(false);
            //var p = ToENU(target.position);
            //var local = rosOrigin == null ? target.rotation : Quaternion.Inverse(rosOrigin.rotation) * target.rotation;
            //var q = Quaternion.AngleAxis(-90, Vector3.up) * local;
            //msg.pose.position.x = p.x; msg.pose.position.y = p.y; msg.pose.position.z = p.z;
            //msg.pose.orientation.x = q.z; msg.pose.orientation.y = -q.x; msg.pose.orientation.z = q.y; msg.pose.orientation.w = -q.w;

            var msg = new PoseStampedMsg(); Stamp(msg.header);
            msg.pose.position = target.transform.position.To<FLU>();
            msg.pose.orientation = target.transform.rotation.To<FLU>();

            if (Send("b3/waypoint", msg)) message = "Target snapshot sent. Pause other velocity publishers to keep position control.";
        }
        void Stamp(HeaderMsg h)
        {
            h.frame_id = frameId;
            long ticks = DateTime.UtcNow.Ticks - 621355968000000000L;
#if ROS2
            h.stamp.sec = (int)(ticks / 10000000);
#else
            h.stamp.sec = (uint)(ticks / 10000000);
#endif
            h.stamp.nanosec = (uint)((ticks % 10000000) * 100);
        }
        public void ApplyBounds()
        {
            var values = new float[7];
            for (int i = 0; i < 7; i++)
                if (!float.TryParse(fields[i].text, NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]) || !float.IsFinite(values[i]))
                { message = "Enter finite numbers using decimal point."; return; }
            var b = new BoundsCommand { min = new[] { values[0], values[1], values[2] }, max = new[] { values[3], values[4], values[5] }, margin = values[6] };
            if (Send("bounds_unity", new StringMsg(JsonUtility.ToJson(b)))) message = "Bounds requested; wait for bounds / applied. Requires DISARMED.";
        }
        public void LoadBounds()
        {
            if (!StatusFresh || Status.box_min == null || Status.box_max == null) { message = "No server bounds received."; return; }
            for (int i = 0; i < 3; i++)
            {
                fields[i].text = Status.box_min[i].ToString("G", CultureInfo.InvariantCulture);
                fields[i + 3].text = Status.box_max[i].ToString("G", CultureInfo.InvariantCulture);
            }
            fields[6].text = Status.boundary_margin.ToString("G", CultureInfo.InvariantCulture);
            message = "Loaded server values. Editing fields does not apply them.";
        }
        void Update()
        {
            if (root == null) return;
            //if (ManualEnabled && (!focused || !StatusFresh)) SetManualEnabled(false);
            if (ManualEnabled && (!focused || (!simulationFlightSticks && !StatusFresh)))
                SetManualEnabled(false); ///
            if (appliedVelocityMode != velocityMode) ApplyVelocityMode();
            //if (ManualEnabled && GlobalYaw && !headingInitialized && StatusFresh && Status.yaw_valid)
            //{ heading.SetHeading(Status.yaw_enu); headingInitialized = true; }
            if (ManualEnabled && GlobalYaw && !headingInitialized)
                TryInitializeHeading();
            bool ready = ManualEnabled && MotionReady && (!GlobalYaw || headingInitialized);
            move.InputEnabled = lift.InputEnabled = heading.InputEnabled = ready;
            double now = Time.realtimeSinceStartupAsDouble;
            if (ready && HasManualInput && now >= nextSend)
            {
                var a = move.Value; var b = lift.Value;
                // Screen up = +X east/forward; screen left = +Y north/left.
                //var v = Vector3.ClampMagnitude(new Vector3(a.y, -a.x, b.y), 1) * Mathf.Min(linearSpeed, Status.max_speed);
                //var cmd = new MotionCommand {
                //    frame_id = frameId, is_body_frame = BodyFrame, use_yaw_rate = !GlobalYaw,
                //    values = new[] { v.x, v.y, v.z, GlobalYaw ? heading.HeadingRadians : -b.x * Mathf.Min(yawRate, Status.max_yaw_rate) }
                //};
                //Send("motion_unity", new StringMsg(JsonUtility.ToJson(cmd)));
                //nextSend = now + 1 / Mathf.Max(1, publishHz);


                float speed = simulationFlightSticks ? linearSpeed : Mathf.Min(linearSpeed, Status.max_speed);

                float rate = simulationFlightSticks ? yawRate : Mathf.Min(yawRate, Status.max_yaw_rate);

                // Screen up = +X east/forward; screen left = +Y north/left.
                var v = Vector3.ClampMagnitude(new Vector3(a.y, -a.x, b.y), 1) * speed;

                float yawValue = GlobalYaw ? heading.HeadingRadians : -b.x * rate;

                var cmd = new MotionCommand
                {
                    frame_id = frameId,
                    is_body_frame = BodyFrame,
                    use_yaw_rate = !GlobalYaw,
                    values = new[] { v.x, v.y, v.z, yawValue }
                };

                if (Send("motion_unity", new StringMsg(JsonUtility.ToJson(cmd))) &&
                    simulationFlightSticks)
                {
                    simulationMotionSent = true;
                    lastMotionBodyFrame = BodyFrame;
                    lastMotionGlobalYaw = GlobalYaw;
                    lastMotionYaw = yawValue;
                }

                nextSend = now + 1 / Mathf.Max(1, publishHz);
            }
            if (simulationFlightSticks && simulationMotionSent &&
                !(ready && HasManualInput))
            {
                StopSimulationSticks();
            }
            // No zero on release, no idle publication, no disable publication.
            if (now >= nextPaint) { Paint(); nextPaint = now + 0.1; }
        }

        void StopSimulationSticks()
        {
            if (!simulationFlightSticks || !simulationMotionSent) return;

            var cmd = new MotionCommand
            {
                frame_id = frameId,
                is_body_frame = lastMotionBodyFrame,
                use_yaw_rate = !lastMotionGlobalYaw,
                values = new[] {
            0f, 0f, 0f,
            lastMotionGlobalYaw ? lastMotionYaw : 0f
        }
            };

            if (Send("motion_unity", new StringMsg(JsonUtility.ToJson(cmd))))
                simulationMotionSent = false;
        }

        void TryInitializeHeading()///
        {
            if (headingInitialized || heading == null) return;

            if (simulationFlightSticks)
            {
                // Assumes this HUD controls the single DroneCommander in the scene.
                var drone = FindFirstObjectByType<DroneCommander>();
                if (drone == null) return;

                Quaternion localRotation = rosOrigin == null
                    ? drone.transform.rotation
                    : Quaternion.Inverse(rosOrigin.rotation) * drone.transform.rotation;

                // Unity yaw: +Z/north = 0 degrees.
                // ENU yaw:   +X/east  = 0 radians, +Y/north = pi/2.
                heading.SetHeading((90f - localRotation.eulerAngles.y) * Mathf.Deg2Rad);
                headingInitialized = true;
                return;
            }

            if (StatusFresh && Status.yaw_valid)
            {
                heading.SetHeading(Status.yaw_enu);
                headingInitialized = true;
            }
        }


        // Add these methods inside the first partial class (Logic file), e.g., right below SendTarget():

        public void LoadTarget()
        {
            if (target == null)
            {
                message = "Assign Target in Inspector to load its transform.";
                return;
            }
            Vector3 pos = rosOrigin == null ? target.position : rosOrigin.InverseTransformPoint(target.position);
            float yaw = (rosOrigin == null ? target.rotation : Quaternion.Inverse(rosOrigin.rotation) * target.rotation).eulerAngles.y;

            targetFields[0].text = pos.x.ToString("G", CultureInfo.InvariantCulture);
            targetFields[1].text = pos.y.ToString("G", CultureInfo.InvariantCulture);
            targetFields[2].text = pos.z.ToString("G", CultureInfo.InvariantCulture);
            targetFields[3].text = yaw.ToString("F1", CultureInfo.InvariantCulture);
            message = "Loaded current Target transform values.";
        }

        public bool ApplyTargetTransform()
        {
            var values = new float[4];
            for (int i = 0; i < 4; i++)
            {
                if (!float.TryParse(targetFields[i].text, NumberStyles.Float, CultureInfo.InvariantCulture, out values[i]) || !float.IsFinite(values[i]))
                {
                    message = "Enter finite numbers for Target using decimal point.";
                    return false;
                }
            }

            Vector3 localPos = new Vector3(values[0], values[1], values[2]);
            Quaternion localRot = Quaternion.Euler(0f, values[3], 0f);

            if (target != null)
            {
                target.position = rosOrigin == null ? localPos : rosOrigin.TransformPoint(localPos);
                target.rotation = rosOrigin == null ? localRot : rosOrigin.rotation * localRot;
            }

            message = "Target position updated locally.";
            return true;
        }

        public void ApplyAndSendTarget()
        {
            if (!ApplyTargetTransform()) return;

            SetManualEnabled(false);
            var msg = new PoseStampedMsg();
            Stamp(msg.header);

            if (target != null)
            {
                msg.pose.position = target.transform.position.To<FLU>();
                msg.pose.orientation = target.transform.rotation.To<FLU>();
            }
            else
            {
                // Fallback if no target Transform is assigned in Inspector: use values directly
                float x = float.Parse(targetFields[0].text, CultureInfo.InvariantCulture);
                float y = float.Parse(targetFields[1].text, CultureInfo.InvariantCulture);
                float z = float.Parse(targetFields[2].text, CultureInfo.InvariantCulture);
                float yaw = float.Parse(targetFields[3].text, CultureInfo.InvariantCulture);

                Vector3 worldPos = rosOrigin == null ? new Vector3(x, y, z) : rosOrigin.TransformPoint(new Vector3(x, y, z));
                Quaternion worldRot = rosOrigin == null ? Quaternion.Euler(0, yaw, 0) : rosOrigin.rotation * Quaternion.Euler(0, yaw, 0);

                msg.pose.position = worldPos.To<FLU>();
                msg.pose.orientation = worldRot.To<FLU>();
            }

            if (Send("b3/waypoint", msg))
                message = "Updated target snapshot sent. Pause other velocity publishers to keep position control.";
        }





        void OnApplicationFocus(bool value) { focused = value; if (!value) SetManualEnabled(false); }
        void OnApplicationPause(bool value) { if (value) SetManualEnabled(false); }
        void OnEnable() { if (root != null) root.SetActive(true); }
        void OnDisable() { SetManualEnabled(false); if (root != null) root.SetActive(false); }
        void OnDestroy() { if (root != null) Destroy(root); if (circleSprite != null) Destroy(circleSprite); if (circleTexture != null) Destroy(circleTexture); }
        void OnDrawGizmosSelected()
        {
            if (Status?.box_min == null || Status.box_max == null) return;
            var a = Status.box_min; var b = Status.box_max;
            var center = new Vector3((float)(a[0] + b[0]), (float)(a[2] + b[2]), (float)(a[1] + b[1])) * 0.5f;
            var size = new Vector3((float)(b[0] - a[0]), (float)(b[2] - a[2]), (float)(b[1] - a[1]));
            var previous = Gizmos.matrix;
            Gizmos.matrix = rosOrigin == null ? Matrix4x4.identity : Matrix4x4.TRS(rosOrigin.position, rosOrigin.rotation, Vector3.one);
            Gizmos.color = Color.cyan; Gizmos.DrawWireCube(center, size); Gizmos.matrix = previous;
        }
    }
}
