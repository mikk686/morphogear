// DroneRigBindings.cs
// Registers everything into the settings menu, including the ArticulationBody
// MASS and IMMOVABLE features. This is the only file you edit to add parameters.
//
// Put this component on the drone root (the object with the root ArticulationBody)
// or anywhere in the scene - references are auto-found.
using System.Collections;
using UnityEngine;

namespace DroneLab
{
    [DisallowMultipleComponent]
    public class DroneRigBindings : MonoBehaviour
    {
        [Header("Targets (auto-found if empty)")]
        public DroneCommander commander;
        [Tooltip("Root ArticulationBody of the drone (isRoot == true).")]
        public ArticulationBody root;
        public Camera viewCamera;

        [Header("Camera view")]
        public Transform followTarget;
        public Vector3 chaseOffset = new Vector3(0f, 2.5f, -6f);
        public float cameraSmooth = 8f;
        public bool lookAtDrone = true;

        [Header("Mass control")]
        [Tooltip("Scales the mass of EVERY body in the chain, preserving their ratios. 1 = as authored.")]
        public float massScale = 1f;

        // ---- runtime
        private ArticulationBody[] _bodies = System.Array.Empty<ArticulationBody>();
        private float[] _authoredMass = System.Array.Empty<float>();
        private Vector3 _spawnPos;
        private Quaternion _spawnRot;
        private bool _spawnCaptured;

        // ==================================================================
        private void Start()
        {
            if (commander == null) commander = FindFirstObjectByType<DroneCommander>(FindObjectsInactive.Include);

            if (root == null)
            {
                var candidate = commander != null
                    ? commander.GetComponentInChildren<ArticulationBody>(true)
                    : FindFirstObjectByType<ArticulationBody>(FindObjectsInactive.Include);
                root = FindRoot(candidate);
            }

            if (root != null)
            {
                _bodies = root.GetComponentsInChildren<ArticulationBody>(true);
                _authoredMass = new float[_bodies.Length];
                for (int i = 0; i < _bodies.Length; i++) _authoredMass[i] = _bodies[i].mass;

                _spawnPos = root.transform.position;
                _spawnRot = root.transform.rotation;
                _spawnCaptured = true;
            }
            else
            {
                Debug.LogWarning("[DroneRigBindings] No ArticulationBody found - rig settings disabled.");
            }

            //if (viewCamera == null) viewCamera = Camera.main;
            //if (followTarget == null && root != null) followTarget = root.transform;
            //else if (followTarget == null && commander != null) followTarget = commander.transform;

            RegisterCommander();
            RegisterRig();
            RegisterPid();
            //RegisterCamera();
            RegisterWorld();
        }

        private void OnDestroy() => DroneSettings.Unbind(this);

        private static ArticulationBody FindRoot(ArticulationBody any)
        {
            if (any == null) return null;
            var b = any;
            while (b != null && !b.isRoot)
            {
                var parent = b.transform.parent != null
                    ? b.transform.parent.GetComponentInParent<ArticulationBody>()
                    : null;
                if (parent == null) break;
                b = parent;
            }
            return b;
        }

        // ==================================================================  COMMANDER
        private void RegisterCommander()
        {
            if (commander == null) return;
            const string cat = "Commander";

            DroneSettings.Enum(cat, "Mode", () => commander.currentMode, v => commander.currentMode = v, this);

            DroneSettings.Header(cat, "Targets", this);
            DroneSettings.Vec3(cat, "Target position", () => commander.targetPosition,
                               v => commander.targetPosition = v, "X,Y,Z", this);
            DroneSettings.Vec3(cat, "Target velocity", () => commander.targetVelocity,
                               v => commander.targetVelocity = v, "X,Y,Z", this);
            DroneSettings.Vec3(cat, "Euler / rates", () => commander.targetEulerOrRates,
                               v => commander.targetEulerOrRates = v, "X,Y,Z", this);
            DroneSettings.Float(cat, "Manual thrust (N)", () => commander.manualThrust,
                                v => commander.manualThrust = v, 0f, 200f, this);

            DroneSettings.Header(cat, "Yaw", this);
            DroneSettings.Bool(cat, "Use explicit yaw", () => commander.useExplicitYaw,
                               v => commander.useExplicitYaw = v, this);
            DroneSettings.Float(cat, "Explicit yaw (deg)", () => commander.explicitYawAngle,
                                v => commander.explicitYawAngle = v, -180f, 180f, this);

            DroneSettings.Header(cat, "Actions", this);
            DroneSettings.Button(cat, "Hold here (pos = current)", () =>
            {
                if (root == null) return;
                commander.targetPosition = root.transform.position;
                commander.targetVelocity = Vector3.zero;
                commander.currentMode = DroneCommander.CommanderMode.PositionControl;
            }, this);
            DroneSettings.Button(cat, "Zero velocity command", () =>
            {
                commander.targetVelocity = Vector3.zero;
                commander.currentMode = DroneCommander.CommanderMode.VelocityControl;
            }, this);
        }

        // ==================================================================  ARTICULATION RIG
        private void RegisterRig()
        {
            if (root == null) return;
            const string cat = "Rig / Body";

            // ---------------- mass ----------------
            DroneSettings.Header(cat, "Mass", this);
            DroneSettings.Readout(cat, "Total chain mass",
                () => TotalMass().ToString("F3") + " kg  (" + _bodies.Length + " bodies)", this);
            DroneSettings.Float(cat, "Root mass (kg)", () => root.mass,
                                v => root.mass = Mathf.Max(0.001f, v), 0.05f, 25f, this);
            DroneSettings.Float(cat, "Mass scale (all bodies)", () => massScale, ApplyMassScale, 0.1f, 5f, this);
            DroneSettings.Readout(cat, "Hover thrust needed",
                () => (TotalMass() * Mathf.Abs(Physics.gravity.y)).ToString("F2") + " N", this);
            DroneSettings.Button(cat, "Restore authored masses", () => ApplyMassScale(1f), this);

            // ---------------- immovable ----------------
            DroneSettings.Header(cat, "Immovable (root only)", this);
            DroneSettings.Bool(cat, "Immovable", () => root.immovable, SetImmovable, this);
            DroneSettings.Readout(cat, "State",
                () => root.immovable ? "PINNED - forces ignored" : "free flight", this);
            DroneSettings.Button(cat, "Pin in mid-air (tune PIDs)", () => SetImmovable(true), this);
            DroneSettings.Button(cat, "Release", () => SetImmovable(false), this);

            // ---------------- solver / damping ----------------
            DroneSettings.Header(cat, "Dynamics", this);
            DroneSettings.Bool(cat, "Use gravity", () => root.useGravity, SetAllUseGravity, this);
            DroneSettings.Float(cat, "Linear damping", () => root.linearDamping, SetAllLinearDamping, 0f, 5f, this);
            DroneSettings.Float(cat, "Angular damping", () => root.angularDamping, SetAllAngularDamping, 0f, 10f, this);
            DroneSettings.Float(cat, "Joint friction", () => root.jointFriction, v => root.jointFriction = v, 0f, 5f, this);
            DroneSettings.Float(cat, "Max linear velocity", () => root.maxLinearVelocity,
                                v => root.maxLinearVelocity = v, 1f, 200f, this);
            DroneSettings.Float(cat, "Max angular velocity", () => root.maxAngularVelocity,
                                v => root.maxAngularVelocity = v, 1f, 100f, this);
            DroneSettings.Int(cat, "Solver iterations", () => root.solverIterations,
                              v => root.solverIterations = Mathf.Max(1, v), 1f, 64f, this);
            DroneSettings.Int(cat, "Solver velocity iterations", () => root.solverVelocityIterations,
                              v => root.solverVelocityIterations = Mathf.Max(0, v), 0f, 32f, this);
            DroneSettings.Bool(cat, "Automatic center of mass", () => root.automaticCenterOfMass,
                               v => root.automaticCenterOfMass = v, this);

            // ---------------- pose ----------------
            DroneSettings.Header(cat, "Pose", this);
            DroneSettings.Readout(cat, "Position", () => Fmt(root.transform.position), this);
            DroneSettings.Readout(cat, "Velocity",
                () => Fmt(root.linearVelocity) + "  " + root.linearVelocity.magnitude.ToString("F2") + " m/s", this);
            DroneSettings.Button(cat, "Teleport to spawn", ResetPose, this);
            DroneSettings.Button(cat, "Capture spawn = here", CaptureSpawn, this);
            DroneSettings.Button(cat, "Freeze motion (immovable blip)", () => StartCoroutine(BlipImmovable()), this);
        }

        // ==================================================================  PID (auto-discovered)
        private void RegisterPid()
        {
            if (commander == null) return;

            // The controller type is not referenced directly, so scan the components next to
            // the commander and expose every Vector3 field whose name contains "pid" or "gain".
            var comps = commander.GetComponents<MonoBehaviour>();
            int total = 0;
            foreach (var c in comps)
            {
                if (c == null || c is DroneCommander || c is DroneRigBindings) continue;
                int n = DroneSettings.BindPidGains(c, "PID gains");
                if (n > 0)
                {
                    Debug.Log($"[DroneRigBindings] Exposed {n} PID gain vectors from {c.GetType().Name}.");
                    total += n;
                }
            }
            if (total == 0)
                Debug.Log("[DroneRigBindings] No PID gain vectors found - add them manually with DroneSettings.Vec3(...).");
        }

        // ==================================================================  CAMERA
        private void RegisterCamera()
        {
            const string cat = "Camera";

            if (viewCamera != null)
            {
                DroneSettings.Float(cat, "Field of view", () => viewCamera.fieldOfView,
                                    v => viewCamera.fieldOfView = v, 30f, 120f, this);
                DroneSettings.Float(cat, "Near clip", () => viewCamera.nearClipPlane,
                                    v => viewCamera.nearClipPlane = v, 0.01f, 2f, this);
                DroneSettings.Float(cat, "Far clip", () => viewCamera.farClipPlane,
                                    v => viewCamera.farClipPlane = v, 100f, 5000f, this);
            }

            DroneSettings.Header(cat, "Chase view", this);
            DroneSettings.Vec3(cat, "Offset", () => chaseOffset, v => chaseOffset = v, "X,Y,Z", this);
            DroneSettings.Float(cat, "Follow smoothing", () => cameraSmooth, v => cameraSmooth = v, 0f, 30f, this);
            DroneSettings.Bool(cat, "Look at drone", () => lookAtDrone, v => lookAtDrone = v, this);
            DroneSettings.Button(cat, "Snap to drone", () =>
            {
                if (viewCamera != null && followTarget != null)
                    viewCamera.transform.position = followTarget.TransformPoint(chaseOffset);
            }, this);
        }

        // ==================================================================  WORLD
        private void RegisterWorld()
        {
            const string cat = "World";
            DroneSettings.Float(cat, "Fixed timestep (s)", () => Time.fixedDeltaTime,
                                v => Time.fixedDeltaTime = Mathf.Clamp(v, 0.001f, 0.05f), 0.002f, 0.02f, this);
            DroneSettings.Float(cat, "Gravity Y", () => Physics.gravity.y,
                                v => Physics.gravity = new Vector3(0f, v, 0f), -20f, 0f, this);
            DroneSettings.Readout(cat, "Physics steps/s",
                () => (1f / Mathf.Max(0.0001f, Time.fixedDeltaTime)).ToString("F0") + " Hz", this);
        }

        // ==================================================================  rig operations
        public float TotalMass()
        {
            float m = 0f;
            for (int i = 0; i < _bodies.Length; i++)
                if (_bodies[i] != null) m += _bodies[i].mass;
            return m;
        }

        /// <summary>Scales every body's mass relative to the authored values (keeps ratios intact).</summary>
        public void ApplyMassScale(float scale)
        {
            massScale = Mathf.Max(0.01f, scale);
            for (int i = 0; i < _bodies.Length; i++)
            {
                if (_bodies[i] == null) continue;
                _bodies[i].mass = Mathf.Max(0.001f, _authoredMass[i] * massScale);
            }
        }

        /// <summary>immovable only has an effect on the articulation ROOT - guarded here.</summary>
        public void SetImmovable(bool value)
        {
            if (root == null) return;
            if (!root.isRoot)
            {
                Debug.LogWarning("[DroneRigBindings] 'immovable' only applies to the articulation root.");
                return;
            }
            root.immovable = value;
            if (!value) root.WakeUp();
        }

        /// <summary>Teleport back to the captured spawn pose with motion killed.</summary>
        public void ResetPose()
        {
            if (root == null || !_spawnCaptured) return;
            StartCoroutine(ResetRoutine());
        }

        private IEnumerator ResetRoutine()
        {
            bool wasImmovable = root.isRoot && root.immovable;
            if (root.isRoot) root.immovable = true;          // kills linear + angular motion
            yield return new WaitForFixedUpdate();

            root.TeleportRoot(_spawnPos, _spawnRot);
            yield return new WaitForFixedUpdate();

            if (root.isRoot) root.immovable = wasImmovable;
            root.WakeUp();

            if (commander != null)
            {
                commander.targetPosition = _spawnPos;
                commander.targetVelocity = Vector3.zero;
                commander.explicitYawAngle = _spawnRot.eulerAngles.y;
            }
        }

        /// <summary>One-frame immovable pulse - the cleanest way to zero an articulation's motion.</summary>
        private IEnumerator BlipImmovable()
        {
            if (root == null || !root.isRoot) yield break;
            bool was = root.immovable;
            root.immovable = true;
            yield return new WaitForFixedUpdate();
            root.immovable = was;
            root.WakeUp();
        }

        /// <summary>Re-capture the spawn pose at the drone's current place.</summary>
        public void CaptureSpawn()
        {
            if (root == null) return;
            _spawnPos = root.transform.position;
            _spawnRot = root.transform.rotation;
            _spawnCaptured = true;
        }

        private void SetAllUseGravity(bool v)
        {
            foreach (var b in _bodies) if (b != null) b.useGravity = v;
        }

        private void SetAllLinearDamping(float v)
        {
            foreach (var b in _bodies) if (b != null) b.linearDamping = v;
        }

        private void SetAllAngularDamping(float v)
        {
            foreach (var b in _bodies) if (b != null) b.angularDamping = v;
        }

        private static string Fmt(Vector3 v) =>
            "(" + v.x.ToString("F2") + ", " + v.y.ToString("F2") + ", " + v.z.ToString("F2") + ")";

        // ==================================================================  chase cam
        private void LateUpdate()
        {
            if (viewCamera == null || followTarget == null || cameraSmooth <= 0f) return;
            Vector3 want = followTarget.TransformPoint(chaseOffset);
            viewCamera.transform.position = Vector3.Lerp(viewCamera.transform.position, want,
                                                         1f - Mathf.Exp(-cameraSmooth * Time.unscaledDeltaTime));
            if (lookAtDrone) viewCamera.transform.LookAt(followTarget.position);
        }
    }
}
