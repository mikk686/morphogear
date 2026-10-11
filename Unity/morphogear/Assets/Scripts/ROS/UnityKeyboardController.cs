using RosMessageTypes.Std;
using System.Threading;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.Controls;
using static UnityEngine.Audio.ControlContext;

public enum ControlMode
{
    Normal,
    WaitingForTeleopDirection,
    TeleopHandControl,
    Flight
}

public enum KeyboardDestination
{
    Unity,
    Robot
}

//[RequireComponent(typeof(Control))]
public class UnifiedKeyboardController : MonoBehaviour
{
    [Header("Unity controller")]
    public Control control;
    public DroneCommander commander;
    public Transform cameraObject;
    public ControlMode _currentMode = ControlMode.Normal;

    [Header("Keyboard destination")]
    [SerializeField]
    private KeyboardDestination startingDestination =
        KeyboardDestination.Unity;

    public KeyboardDestination CurrentDestination { get; private set; }

    [Header("ROS")]
    public string keyboardTopic = "/keyboard_state";
    public string takeoffTopic = "takeoff_unity";
    public string landingTopic = "land_unity";
    public string manualControlTopic = "morphogear_sudo_manual";
    public string commandTopic = "morphogear_sudo_cmd";

    public string obstacleTopic = "nn/enable";

    [Min(1f)]
    public float sendRateHz = 15f;

    private ROSConnection ros;
    public ArticulationBody MG;

    public Vector3 tpPose = new Vector3(0f, 0.6f, 0f);
    private Vector3 lastPose;

    // Accessed by ROS callbacks; consumed on Unity's Update thread.
    private int pendingTakeoff;
    private int pendingLanding;

    // Robot keyboard sender state.
    private int pendingPressMask;
    private int lastSentHeldMask;
    private bool idleSent;
    private float nextSendTime;

    private bool transitionInProgress;

    /// <summary>
    /// new
    /// </summary>
    /// 

    private bool managedByGameManager;
    private bool rosEnabled;
    private bool acceptFlightTopics;
    private bool inputBlocked;

    // Wire order:
    // w,a,s,d,q,e,z,c,x,space,digit1,digit2,digit3,digit4
    private const int FieldCount = 14;

    private void Awake()
    {
        //if (control == null)
        //    control = GetComponent<Control>();

        //MG = GetComponent<ArticulationBody>();
    }

    //private void Start()
    //{
    //    ros = ROSConnection.GetOrCreateInstance();

    //    ros.RegisterPublisher<StringMsg>(keyboardTopic);
    //    ros.RegisterPublisher<BoolMsg>(manualControlTopic);
    //    ros.RegisterPublisher<StringMsg>(commandTopic);

    //    ros.Subscribe<BoolMsg>(takeoffTopic, OnTakeoffMessage);
    //    ros.Subscribe<BoolMsg>(landingTopic, OnLandingMessage);

    //    CurrentDestination = startingDestination;
    //    ResetSender();
    //}

    private void Start()
    {
        // With a GameManager, it configures us before Start().
        // Without one, preserve standalone behavior.
        if (!managedByGameManager)
        {
            CurrentDestination = startingDestination;
            rosEnabled = true;
            acceptFlightTopics = true;
            EnsureRos();
        }

        ResetSender();
    }


    public void Update()
    {
        if (!acceptFlightTopics)
        {
            Interlocked.Exchange(ref pendingTakeoff, 0);
            Interlocked.Exchange(ref pendingLanding, 0);
        }
        // Process ROS requests here so simulation actions start on Unity's thread.
        bool takeoffRequested = Interlocked.Exchange(ref pendingTakeoff, 0) != 0;
        bool landingRequested = Interlocked.Exchange(ref pendingLanding, 0) != 0;

        if (!transitionInProgress)
        {
            if (landingRequested && _currentMode == ControlMode.Flight)
                _ = PrepareAndLanding();
            else if (takeoffRequested && _currentMode != ControlMode.Flight)
                _ = PrepareAndTakeOff();
        }
        UpdateMotionControl();
        Keyboard kb = Keyboard.current;


        if (kb.bKey.wasPressedThisFrame)
        {
            Debug.Log("Sit down");
            SendStringCommand("sitdown");
        }

        if(kb.minusKey.wasPressedThisFrame)
        {
            Debug.Log("turn off Obstacle Avoidance");
            ActivateNN(false);
        }
        else if (kb.equalsKey.wasPressedThisFrame)
        {
            Debug.Log("turn off Obstacle Avoidance");
            ActivateNN(true);
        }

        //// Switching destination is available even during a flight transition.
        //if (kb != null && kb.f1Key.wasPressedThisFrame)
        //{
        //    SetDestination(KeyboardDestination.Unity);
        //    return;
        //}

        //if (kb != null && kb.f2Key.wasPressedThisFrame)
        //{
        //    SetDestination(KeyboardDestination.Robot);
        //    return;
        //}
        if (!managedByGameManager && kb != null && kb.f1Key.wasPressedThisFrame)
        {
            SetDestination(KeyboardDestination.Unity);
            return;
        }

        if (!managedByGameManager && kb != null && kb.f2Key.wasPressedThisFrame)
        {
            SetDestination(KeyboardDestination.Robot);
            return;
        }
        if (inputBlocked) return;

        //if (CurrentDestination == KeyboardDestination.Robot)
        //{
        //    UpdateRobotSender(kb);
        //    return;
        //}

        if (CurrentDestination == KeyboardDestination.Robot && rosEnabled)
        {
            UpdateRobotSender(kb);
            return;
        }

        // While a takeoff/landing sequence is running, do not start
        // another local control action.
        if (transitionInProgress || kb == null)
            return;

        UpdateUnityController(kb);



    }

    private void ActivateNN(bool run)
    {
        if (!rosEnabled) return;
        EnsureRos();
        ros.Publish(obstacleTopic, new BoolMsg(run));
    }



    private void EnsureRos()
    {
        if (!rosEnabled || ros != null)
            return;

        ros = ROSConnection.GetOrCreateInstance();

        ros.RegisterPublisher<StringMsg>(keyboardTopic);
        ros.RegisterPublisher<BoolMsg>(manualControlTopic);
        ros.RegisterPublisher<StringMsg>(commandTopic);

        ros.RegisterPublisher<BoolMsg>(obstacleTopic);

        ros.Subscribe<BoolMsg>(takeoffTopic, OnTakeoffMessage);
        ros.Subscribe<BoolMsg>(landingTopic, OnLandingMessage);
        ros.Subscribe<StringMsg>(motionTopic, OnMotionMessage);
    }

    public void ConfigureManagedMode(
    bool useRos,
    KeyboardDestination destination,
    bool simulateFlightTopics)
    {
        managedByGameManager = true;

        // SetDestination sends a neutral packet when leaving Robot mode.
        // Do that BEFORE disabling ROS for Game mode.
        if (CurrentDestination != destination)
            SetDestination(destination);

        rosEnabled = useRos;
        acceptFlightTopics = simulateFlightTopics;

        Interlocked.Exchange(ref pendingTakeoff, 0);
        Interlocked.Exchange(ref pendingLanding, 0);

        if (rosEnabled)
            EnsureRos();
    }

    public void SetInputBlocked(bool blocked)
    {
        if (inputBlocked == blocked)
            return;

        inputBlocked = blocked;

        if (blocked && CurrentDestination == KeyboardDestination.Robot)
            PublishNeutral();

        ResetSender();
    }

    //private void OnTakeoffMessage(BoolMsg message)
    //{
    //    if (message.data)
    //        Interlocked.Exchange(ref pendingTakeoff, 1);
    //}

    //private void OnLandingMessage(BoolMsg message)
    //{
    //    if (message.data)
    //        Interlocked.Exchange(ref pendingLanding, 1);
    //}
    private void OnTakeoffMessage(BoolMsg message)
    {
        if (acceptFlightTopics && message.data)
            Interlocked.Exchange(ref pendingTakeoff, 1);
    }

    private void OnLandingMessage(BoolMsg message)
    {
        if (acceptFlightTopics && message.data)
            Interlocked.Exchange(ref pendingLanding, 1);
    }


    public string motionTopic = "motion_unity";
    public string motionFrameId = "map";

    [System.Serializable]
    private class MotionCommand
    {
        public string frame_id;
        public float[] values;
        public bool is_body_frame;
        public bool use_yaw_rate;
    }

    private readonly object motionLock = new object();
    private string pendingMotionJson;

    private bool motionControlActive;
    private bool motionUsesYawRate;
    private float unityYawRateDegPerSecond;
    private bool previousUseExplicitYaw;


    private void OnMotionMessage(StringMsg message)
    {
        // Keep only the newest command. Apply it on Unity's Update thread.
        lock (motionLock)
            pendingMotionJson = message.data;
    }

    private void UpdateMotionControl()
    {
        string json;

        lock (motionLock)
        {
            json = pendingMotionJson;
            pendingMotionJson = null;
        }

        // A command received outside flight must not be replayed after takeoff.
        if (_currentMode != ControlMode.Flight || transitionInProgress)
        {
            if (motionControlActive)
            {
                commander.useExplicitYaw = previousUseExplicitYaw;
                motionControlActive = false;
                unityYawRateDegPerSecond = 0f;
            }

            return;
        }

        if (json != null)
        {
            MotionCommand motion;

            try
            {
                motion = JsonUtility.FromJson<MotionCommand>(json);
            }
            catch (System.ArgumentException)
            {
                return;
            }

            if (motion == null ||
                motion.frame_id != motionFrameId ||
                motion.values == null ||
                motion.values.Length != 4)
                return;

            if (!motionControlActive)
                previousUseExplicitYaw = commander.useExplicitYaw;

            float vx = motion.values[0];
            float vy = motion.values[1];
            float vz = motion.values[2];
            float yaw = motion.values[3];

            // ROS body FLU: forward, left, up.
            // Unity drone local axes: right, up, forward.
            // ROS world ENU: east, north, up.
            //Vector3 unityVelocity = motion.is_body_frame
            //    ? new Vector3(-vy, vz, vx)
            //    : new Vector3(vx, vz, vy);


            Vector3 unityVelocity = new Vector3(-vy, vz, vx);

            commander.targetVelocity = unityVelocity;
            commander.currentMode = motion.is_body_frame
                ? DroneCommander.CommanderMode.LocalVelocityControl
                : DroneCommander.CommanderMode.VelocityControl;

            // DroneCommander exposes an explicit yaw ANGLE, not a yaw-rate
            // command. Integrate rate into that angle each Update below.
            if (motion.use_yaw_rate)
            {
                if (!motionControlActive || !motionUsesYawRate)
                    commander.explicitYawAngle = commander.transform.eulerAngles.y;

                // ENU positive yaw is CCW; Unity positive Y rotation is the
                // opposite direction under the stated axis alignment.
                unityYawRateDegPerSecond = -yaw * Mathf.Rad2Deg;
            }
            else
            {
                unityYawRateDegPerSecond = 0f;
                commander.explicitYawAngle =
                    Mathf.Repeat(- yaw * Mathf.Rad2Deg, 360f);
            }

            commander.useExplicitYaw = true;
            motionUsesYawRate = motion.use_yaw_rate;
            motionControlActive = true;
        }

        if (motionControlActive && motionUsesYawRate)
            commander.explicitYawAngle +=
                unityYawRateDegPerSecond * Time.deltaTime;
    }




    /// <summary>
    /// Can also be called from a UI button or another component.
    /// </summary>
    public void SetDestination(KeyboardDestination destination)
    {
        if (CurrentDestination == destination)
            return;

        if (CurrentDestination == KeyboardDestination.Robot)
        {
            // Explicitly release robot keyboard input before leaving robot mode.
            PublishNeutral();
        }
        else
        {
            // Do not leave an in-progress Unity control action running
            // after keyboard authority moves to the robot.
            control.CancelActiveAction();
        }

        CurrentDestination = destination;
        ResetSender();

        Debug.Log($"Keyboard destination: {destination}");
    }

    private void ResetSender()
    {
        pendingPressMask = 0;
        lastSentHeldMask = 0;
        idleSent = false;
        nextSendTime = 0f;
    }

    private void UpdateRobotSender(Keyboard kb)
    {
        // Read press edges EVERY Update, not only on send ticks.
        // Digit keys retain the original wire format: press bit only.
        int heldMask = 0;

        if (kb != null)
        {
            AddKey(kb.wKey, 0, ref heldMask);
            AddKey(kb.aKey, 1, ref heldMask);
            AddKey(kb.sKey, 2, ref heldMask);
            AddKey(kb.dKey, 3, ref heldMask);
            AddKey(kb.qKey, 4, ref heldMask);
            AddKey(kb.eKey, 5, ref heldMask);
            AddKey(kb.zKey, 6, ref heldMask);
            AddKey(kb.cKey, 7, ref heldMask);
            AddKey(kb.xKey, 8, ref heldMask);
            AddKey(kb.spaceKey, 9, ref heldMask);

            AddPressOnly(kb.digit1Key, 10);
            AddPressOnly(kb.digit2Key, 11);
            AddPressOnly(kb.digit3Key, 12);
            AddPressOnly(kb.digit4Key, 13);
        }

        // Nothing to send while idle, except the first neutral message
        // or the message that releases previously held keys.
        bool needsMessage =
            heldMask != 0 ||
            pendingPressMask != 0 ||
            heldMask != lastSentHeldMask ||
            !idleSent;

        if (!needsMessage || Time.unscaledTime < nextSendTime)
            return;

        ros.Publish(keyboardTopic,
            new StringMsg(BuildPayload(heldMask, pendingPressMask)));

        lastSentHeldMask = heldMask;
        pendingPressMask = 0;
        idleSent = heldMask == 0;
        nextSendTime = Time.unscaledTime + 1f / Mathf.Max(1f, sendRateHz);
    }

    private void AddKey(KeyControl key, int index, ref int heldMask)
    {
        if (key.isPressed)
            heldMask |= 1 << index;

        if (key.wasPressedThisFrame)
            pendingPressMask |= 1 << index;
    }

    private void AddPressOnly(KeyControl key, int index)
    {
        if (key.wasPressedThisFrame)
            pendingPressMask |= 1 << index;
    }

    private static string BuildPayload(int heldMask, int pressMask)
    {
        // 14 two-character fields separated by 13 commas.
        char[] chars = new char[FieldCount * 3 - 1];

        for (int i = 0; i < FieldCount; i++)
        {
            int offset = i * 3;
            chars[offset] = (heldMask & (1 << i)) != 0 ? '1' : '0';
            chars[offset + 1] = (pressMask & (1 << i)) != 0 ? '1' : '0';

            if (i < FieldCount - 1)
                chars[offset + 2] = ',';
        }

        return new string(chars);
    }

    private void PublishNeutral()
    {
        if (ros != null)
            ros.Publish(keyboardTopic, new StringMsg(BuildPayload(0, 0)));
    }

    public void SetManualControl(bool manual)
    {
        if (!rosEnabled) return;
        EnsureRos();
        ros.Publish(manualControlTopic, new BoolMsg(manual));
    }

    public void SendStringCommand(string cmd)
    {
        if (!rosEnabled) return;
        EnsureRos();
        ros.Publish(commandTopic, new StringMsg(cmd));
    }
    //public void SetManualControl(bool manual)
    //{
    //    if (ros != null)
    //        ros.Publish(manualControlTopic, new BoolMsg(manual));
    //}

    //public void SendStringCommand(string cmd)
    //{
    //    if (ros != null)
    //        ros.Publish(commandTopic, new StringMsg(cmd));
    //}

    // ---------------------------------------------------------------------
    // Existing Unity-side keyboard behavior
    // ---------------------------------------------------------------------

    private void UpdateUnityController(Keyboard kb)
    {
        if (kb.backquoteKey.wasPressedThisFrame)
        {
            Debug.Log("Toggled Manual Control Rights");
            control.CancelActiveAction();
            control.controlRights = !control.controlRights;
        }

        if (kb.digit4Key.wasPressedThisFrame)
        {
            control.CancelActiveAction();
            control.controlRights = true;

            if (_currentMode == ControlMode.Normal)
            {
                _currentMode = ControlMode.WaitingForTeleopDirection;
                Debug.Log("Entered Teleop Mode. Waiting for direction (W/A/S/D)...");
            }
            else
            {
                cameraObject.transform.localPosition =
                    new Vector3(0f, 0.3f, -1.2f);
                cameraObject.transform.localRotation =
                    Quaternion.Euler(10f, 0f, 0f);

                _currentMode = ControlMode.Normal;
                control.ExecuteInitialPosition(control.resetDir, 20f);
                Debug.Log("Exited Teleop Mode. Returning to Initial Position.");
            }

            return;
        }

        if (kb.tKey.wasPressedThisFrame && _currentMode != ControlMode.Flight)
            _ = PrepareAndTakeOff();

        if (kb.gKey.wasPressedThisFrame && _currentMode == ControlMode.Flight)
            _ = PrepareAndLanding();

        if (kb.rKey.wasPressedThisFrame)
            MG.TeleportRoot(tpPose, Quaternion.identity);

        switch (_currentMode)
        {
            case ControlMode.Normal:
                if (control.controlRights) ManualController(kb);
                break;

            case ControlMode.WaitingForTeleopDirection:
                if (control.controlRights) TeleopDirectionController(kb);
                break;

            case ControlMode.TeleopHandControl:
                if (control.controlRights) TeleopHandController(kb);
                break;

            case ControlMode.Flight:
                if (!motionControlActive)
                    ManualFlight(kb);
                break;
        }
    }

    private async Awaitable PrepareAndTakeOff()
    {
        if (transitionInProgress)
            return;

        transitionInProgress = true;

        try
        {
            cameraObject.transform.localPosition = new Vector3(0f, 1f, -2f);
            cameraObject.transform.localRotation = Quaternion.Euler(20f, 0f, 0f);

            //control.ExecuteInitialPosition(control.resetDir, 20f);
            //await Awaitable.WaitForSecondsAsync(5f);

            //control.ExecuteSitDown(5f);
            //await Awaitable.WaitForSecondsAsync(5f);

            //control.controlRights = false;
            commander.currentMode =
                DroneCommander.CommanderMode.PositionControl;
            commander.targetPosition = transform.position + 2 * Vector3.up;

            await Awaitable.WaitForSecondsAsync(3f);

            _currentMode = ControlMode.Flight;
            lastPose = commander.targetPosition;
        }
        finally
        {
            transitionInProgress = false;
        }
    }

    private async Awaitable PrepareAndLanding()
    {
        if (transitionInProgress)
            return;

        transitionInProgress = true;

        try
        {
            _currentMode = ControlMode.Normal;
            lastPose = transform.position;

            commander.currentMode =
                DroneCommander.CommanderMode.VelocityControl;
            commander.targetVelocity = Vector3.down;

            await Awaitable.WaitForSecondsAsync(5f);

            //control.controlRights = true;
            //control.ExecuteInitialPosition(control.resetDir, 20f);

            commander.manualThrust = 0;
            commander.targetPosition = Vector3.zero;
            commander.targetVelocity = Vector3.zero;
            commander.targetEulerOrRates = Vector3.zero;
            commander.currentMode =
                DroneCommander.CommanderMode.AttitudeControl;
        }
        finally
        {
            transitionInProgress = false;
        }
    }

    private void ManualFlight(Keyboard kb)
    {
        if (kb.anyKey.wasReleasedThisFrame)
            lastPose = MG.transform.position;

        if (kb.anyKey.isPressed)
        {
            commander.currentMode =
                DroneCommander.CommanderMode.LocalVelocityControl;

            Vector3 input = new Vector3(
                (kb.dKey.isPressed ? 1 : 0) - (kb.aKey.isPressed ? 1 : 0),
                (kb.iKey.isPressed ? 1 : 0) - (kb.kKey.isPressed ? 1 : 0),
                (kb.wKey.isPressed ? 1 : 0) - (kb.sKey.isPressed ? 1 : 0));

            input.Normalize();
            commander.targetVelocity = input;
        }
        else
        {
            commander.targetPosition = lastPose;
            commander.currentMode =
                DroneCommander.CommanderMode.PositionControl;
        }

        if (kb.eKey.isPressed)
            commander.explicitYawAngle += 1f;

        if (kb.qKey.isPressed)
            commander.explicitYawAngle -= 1f;

        if (kb.xKey.wasPressedThisFrame)
            commander.explicitYawAngle = 0f;
    }

    private void ManualController(Keyboard kb)
    {
        if (kb.xKey.wasPressedThisFrame)
            control.ExecuteInitialPosition(control.resetDir, 20f);
        else if (kb.wKey.isPressed)
            control.ExecuteStepAction(1, 12f);
        else if (kb.sKey.isPressed)
            control.ExecuteStepAction(3, 12f);
        else if (kb.dKey.isPressed)
            control.ExecuteStepAction(2, 12f);
        else if (kb.aKey.isPressed)
            control.ExecuteStepAction(4, 12f);
        else if (kb.eKey.wasPressedThisFrame)
        {
            control.ExecuteRotateBase(control._rotLook, 5f);
            control._rotLook = control._rotLook != 0 ? 0 : control.rotLook;
        }
        else if (kb.qKey.wasPressedThisFrame)
        {
            control.ExecuteRotateBase(-control._rotLook, 5f);
            control._rotLook = control._rotLook != 0 ? 0 : control.rotLook;
        }
        else if (kb.cKey.wasPressedThisFrame)
            control.ExecuteRotatePosition(-control.rotStep, 20f);
        else if (kb.zKey.wasPressedThisFrame)
            control.ExecuteRotatePosition(control.rotStep, 20f);

        if (kb.digit1Key.wasPressedThisFrame)
        {
            control.selectedGait = GaitType.Trot;
            cameraObject.transform.localPosition =
                new Vector3(0f, 0.3f, -1.2f);
            cameraObject.transform.localRotation =
                Quaternion.Euler(10f, 0f, 0f);
        }

        if (kb.digit2Key.wasPressedThisFrame)
        {
            control.selectedGait = GaitType.Canter;
            cameraObject.transform.localPosition =
                new Vector3(1f, 0.5f, -1f);
            cameraObject.transform.localRotation =
                Quaternion.Euler(15f, -45f, 0f);
        }

        if (kb.digit3Key.wasPressedThisFrame)
        {
            control.selectedGait = GaitType.Gallop;
            cameraObject.transform.localPosition =
                new Vector3(0f, 0.5f, -1.5f);
            cameraObject.transform.localRotation =
                Quaternion.Euler(15f, 0f, 0f);
        }
    }

    private void TeleopDirectionController(Keyboard kb)
    {
        int direction = 0;

        if (kb.wKey.wasPressedThisFrame) direction = 1;
        else if (kb.dKey.wasPressedThisFrame) direction = 2;
        else if (kb.sKey.wasPressedThisFrame) direction = 3;
        else if (kb.aKey.wasPressedThisFrame) direction = 4;

        control.resetDir = (direction + 2) % 4 + 1;

        if (direction != 0)
        {
            Debug.Log(
                $"Direction {direction} chosen. Transitioning to Teleoperation pose.");

            _currentMode = ControlMode.TeleopHandControl;
            control.ExecuteTeleoperationPosition(direction, 30f);

            Vector3 baseOffset = new Vector3(0f, 0.7f, 0.2f);
            float angleY = (direction - 1) * 90f;
            Quaternion rotationY = Quaternion.Euler(0, angleY, 0);

            cameraObject.transform.localPosition = rotationY * baseOffset;
            cameraObject.transform.localRotation =
                rotationY * Quaternion.Euler(90f, 0f, 0f);
        }
    }

    private void TeleopHandController(Keyboard kb)
    {
        if (kb.qKey.isPressed) control.Articulate(0, true);
        if (kb.wKey.isPressed) control.Articulate(0, false);
        if (kb.aKey.isPressed) control.Articulate(1, true);
        if (kb.zKey.isPressed) control.Articulate(1, false);
        if (kb.sKey.isPressed) control.Articulate(2, true);
        if (kb.xKey.isPressed) control.Articulate(2, false);

        if (kb.spaceKey.wasPressedThisFrame)
            _ = control.Grab();
    }
}