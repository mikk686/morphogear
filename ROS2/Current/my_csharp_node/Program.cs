// =============================================================================
//  morphogear — ROS 2 node (Rcl.NET / rclnet)
//
//  INPUT:  /keyboard_state          std_msgs/String   <- Unity per-frame snapshot
//          morphogear_sudo_cmd      std_msgs/String   <- legacy string commands
//          /theta_angles            std_msgs/Int8MultiArray  <- KMPC trajectories
//          morphogear_sudo_manual   std_msgs/Bool     <- manual/internal switch
//
//  OUTPUT: /angles_control          std_msgs/Int8MultiArray  @ 20 Hz
//          /request_angles          std_msgs/Empty
//          /robot_state             std_msgs/String          @ 10 Hz  (NEW)
// =============================================================================

using Rcl;
using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

using RosString         = Rosidl.Messages.Std.String;
using RosBool           = Rosidl.Messages.Std.Bool;
using RosInt8MultiArray = Rosidl.Messages.Std.Int8MultiArray;
using RosEmpty          = Rosidl.Messages.Std.Empty;
using RosMultiArrayLayout    = Rosidl.Messages.Std.MultiArrayLayout;
using RosMultiArrayDimension = Rosidl.Messages.Std.MultiArrayDimension;
using RosInt32 = Rosidl.Messages.Std.Int32;

namespace ConsoleApplication
{
    public enum GaitType    { Trot, Canter, Gallop }
    public enum ControlMode { ManualController, TeleopDirection, TeleopHandControl }

    // =========================================================================
    //  Keyboard
    //
    //  Wire format (14 comma-separated fields, 2 chars each):
    //      w,a,s,d,q,e,z,c,x,space,digit1,digit2,digit3,digit4
    //      [0] = '1' while held   [1] = '1' on press frame
    // =========================================================================
    public sealed class Keyboard
    {
        public Key w = new(), a = new(), s = new(), d = new();
        public Key q = new(), e = new(), z = new(), c = new(), x = new();
        public Key digit1 = new(), digit2 = new(), digit3 = new(), digit4 = new();
        public Key space = new();

        public sealed class Key
        {
            public bool isPressed;
            public bool wasPressedThisFrame;
        }

        public static Keyboard Parse(string payload)
        {
            var kb = new Keyboard();
            if (string.IsNullOrEmpty(payload)) return kb;
            var parts = payload.Split(',');
            if (parts.Length < 14) return kb;

            void Apply(Key k, string p)
            {
                if (p.Length < 2) return;
                k.isPressed           = p[0] == '1';
                k.wasPressedThisFrame = p[1] == '1';
            }

            Apply(kb.w,      parts[0]);  Apply(kb.a,      parts[1]);
            Apply(kb.s,      parts[2]);  Apply(kb.d,      parts[3]);
            Apply(kb.q,      parts[4]);  Apply(kb.e,      parts[5]);
            Apply(kb.z,      parts[6]);  Apply(kb.c,      parts[7]);
            Apply(kb.x,      parts[8]);  Apply(kb.space,  parts[9]);
            Apply(kb.digit1, parts[10]); Apply(kb.digit2, parts[11]);
            Apply(kb.digit3, parts[12]); Apply(kb.digit4, parts[13]);
            return kb;
        }
    }

    // =========================================================================
    //  CommunicatorDotNet
    // =========================================================================
    public class CommunicatorDotNet
    {
        public delegate void   SetAngle(sbyte value, int index);
        public static SetAngle SetAngleDelegate = null!;

        public delegate sbyte  GetAngle(int index);
        public static GetAngle GetAngleDelegate = null!;

        public delegate void   Request_KMPC(RosEmpty msg);
        public static Request_KMPC Request_KMPC_Delegate = null!;

        private List<sbyte> controlAngles;
        private List<sbyte> stateAngles;
        private bool controlType = false;

        private readonly RclContext ctx;
        private readonly IRclNode   node;

        private readonly IRclPublisher<RosEmpty>          _KMPCPub;
        private readonly IRclPublisher<RosInt8MultiArray> _anglePub;
        private readonly IRclPublisher<RosString>         _statePub;     // NEW
        private readonly RosInt8MultiArray                _anglePubMsg = new();

        private static Control control = null!;

        private Keyboard    _kb          = new();
        private ControlMode _currentMode = ControlMode.ManualController;

        private CommunicatorDotNet(RclContext context)
        {
            ctx  = context;
            node = ctx.CreateNode("morphogear");
            control = new Control();

            controlAngles = new List<sbyte> { 0,0,0,0,0,0,0,0,0,0,0,0 };
            stateAngles   = new List<sbyte> { 0,0,0,0,0,0,0,0,0,0,0,0 };

            SetAngleDelegate      = SetControlAngle;
            GetAngleDelegate      = GetStateAngle;
            Request_KMPC_Delegate = Request_KMPC_Angles;

            _KMPCPub  = node.CreatePublisher<RosEmpty>("/request_angles");
            _anglePub = node.CreatePublisher<RosInt8MultiArray>("/angles_control_mg");
            _statePub = node.CreatePublisher<RosString>("/robot_state");          // NEW
        }

        public static async Task Main(string[] args)
        {
            await using var ctx = new RclContext(args);
            var talker = new CommunicatorDotNet(ctx);
            await talker.RunAsync();
        }

        private async Task RunAsync()
        {
            using var cts = new CancellationTokenSource();
            Console.CancelKeyPress += (_, e) => { e.Cancel = true; cts.Cancel(); };
            Console.WriteLine("morphogear node started. Mode: ManualController (Trot).");

            await Task.WhenAll(
                ListenCmdAsync(cts.Token),
                ListenKeyboardAsync(cts.Token),
                ListenManualAsync(cts.Token),
                ListenKMPCAsync(cts.Token),
                PublishTimerLoopAsync(cts.Token),
                PublishStateLoopAsync(cts.Token));           // NEW
        }

        private async Task ListenCmdAsync(CancellationToken token)
        {
            using var sub = node.CreateSubscription<RosString>("morphogear_sudo_cmd");
            await foreach (var msg in sub.ReadAllAsync(token))
                control.ExecuteCMD(msg.Data);
        }

        private async Task ListenKeyboardAsync(CancellationToken token)
        {
            using var sub = node.CreateSubscription<RosString>("/keyboard_state");
            await foreach (var msg in sub.ReadAllAsync(token))
            {
                _kb = Keyboard.Parse(msg.Data);
                switch (_currentMode)
                {
                    case ControlMode.ManualController:  ManualController(_kb);          break;
                    case ControlMode.TeleopDirection:   TeleopDirectionController(_kb); break;
                    case ControlMode.TeleopHandControl: TeleopHandController(_kb);      break;
                }
            }
        }

        private async Task ListenManualAsync(CancellationToken token)
        {
            using var sub = node.CreateSubscription<RosBool>("morphogear_sudo_manual");
            await foreach (var msg in sub.ReadAllAsync(token))
            {
                controlType = msg.Data;
                Console.WriteLine(msg.Data ? "Manual Unity Control" : "Internal Control");
            }
        }

        // private async Task ListenKMPCAsync(CancellationToken token)
        // {
        //     using var sub = node.CreateSubscription<RosInt8MultiArray>("/theta_angles");
        //     await foreach (var msg in sub.ReadAllAsync(token))
        //         control.MoveRobotBySequence(new List<sbyte>(msg.Data));
        // }

        // Updated KMPC listener:
        private async Task ListenKMPCAsync(CancellationToken token)
        {
            using var sub = node.CreateSubscription<RosInt8MultiArray>("/theta_angles");
            await foreach (var msg in sub.ReadAllAsync(token))
                control.MoveRobotBySequence(new List<sbyte>(msg.Data), trajectorySize);
        }
        // Add this field to your class:
        private int trajectorySize = 200; // Default size

        // Add this subscriber to listen for trajectory size updates:
        private async Task ListenTrajectorySizeAsync(CancellationToken token)
        {
            using var sub = node.CreateSubscription<RosInt32>("/trajectory_size");
            await foreach (var msg in sub.ReadAllAsync(token))
            {
                trajectorySize = msg.Data;
                Console.WriteLine($"Trajectory size updated to: {trajectorySize}");
            }
        }

        private async Task PublishTimerLoopAsync(CancellationToken token)
        {
            using var timer = ctx.CreateTimer(node.Clock, TimeSpan.FromSeconds(0.05));
            while (!token.IsCancellationRequested)
            {
                try
                {
                    await timer.WaitOneAsync(false, token);
                    if (!controlType) PublishAngles();
                }
                catch (OperationCanceledException) { break; }
            }
        }

        // =====================================================================
        //  NEW — State telemetry @ 10 Hz
        //
        //  Wire format (CSV):  gait,mode,busy
        //      gait:  "Trot" | "Canter" | "Gallop"
        //      mode:  "ManualController" | "TeleopDirection" | "TeleopHandControl"
        //      busy:  "1" if !controlRights (action in progress), "0" if idle
        // =====================================================================
        private async Task PublishStateLoopAsync(CancellationToken token)
        {
            using var timer = ctx.CreateTimer(node.Clock, TimeSpan.FromSeconds(0.1));
            while (!token.IsCancellationRequested)
            {
                try
                {
                    await timer.WaitOneAsync(false, token);
                    PublishState();
                }
                catch (OperationCanceledException) { break; }
            }
        }

        private void PublishState()
        {
            string gait = control.selectedGait.ToString();
            string mode = _currentMode.ToString();
            string busy = control.controlRights ? "0" : "1";
            string payload = $"{gait},{mode},{busy}";
            _statePub.Publish(new RosString { Data = payload });
        }

        private void PublishAngles()
        {
            var msg = new RosInt8MultiArray
            {
                Data = controlAngles.ToArray(),
                Layout = new RosMultiArrayLayout
                {
                    DataOffset = 0,
                    Dim = new List<RosMultiArrayDimension>
                    {
                        new RosMultiArrayDimension
                        {
                            Label  = "joints",
                            Size   = (uint)controlAngles.Count,
                            Stride = (uint)controlAngles.Count
                        }
                    }.ToArray()
                }
            };
            _anglePub.Publish(msg);
        }

        public void Request_KMPC_Angles(RosEmpty msg) => _KMPCPub.Publish(msg);

        // =====================================================================
        //  Mode 1 — Manual controller (priority command launch)
        // =====================================================================
        void ManualController(Keyboard kb)
        {
                 if (kb.x.wasPressedThisFrame) { control.ExecuteInitialPosition(control.resetDir, 20f); }
            else if (kb.w.isPressed)           { control.ExecuteStepAction(1, 12f); }
            else if (kb.s.isPressed)           { control.ExecuteStepAction(3, 12f); }
            else if (kb.d.isPressed)           { control.ExecuteStepAction(2, 12f); }
            else if (kb.a.isPressed)           { control.ExecuteStepAction(4, 12f); }

            else if (kb.e.wasPressedThisFrame)
            {
                control.ExecuteRotateBase(control._rotLook, 5f);
                control._rotLook = (control._rotLook != 0) ? 0 : control.rotLook;
            }
            else if (kb.q.wasPressedThisFrame)
            {
                control.ExecuteRotateBase(-control._rotLook, 5f);
                control._rotLook = (control._rotLook != 0) ? 0 : control.rotLook;
            }

            else if (kb.c.wasPressedThisFrame) { control.ExecuteRotatePosition(-control.rotStep, 20f); }
            else if (kb.z.wasPressedThisFrame) { control.ExecuteRotatePosition(control.rotStep, 20f); }

            // Gait selection
            if (kb.digit1.wasPressedThisFrame) { control.selectedGait = GaitType.Trot;   Console.WriteLine("Gait: Trot");   }
            if (kb.digit2.wasPressedThisFrame) { control.selectedGait = GaitType.Canter; Console.WriteLine("Gait: Canter"); }
            if (kb.digit3.wasPressedThisFrame) { control.selectedGait = GaitType.Gallop; Console.WriteLine("Gait: Gallop"); }

            // NEW — digit4 enters TeleopDirection mode
            if (kb.digit4.wasPressedThisFrame)
            {
                control.CancelActiveAction();
                control.controlRights = true;
                _currentMode = ControlMode.TeleopDirection;
                Console.WriteLine("Entered Teleop Mode. Waiting for direction (W/A/S/D)...");
            }
        }

        // =====================================================================
        //  Mode 2 — Teleop direction selection
        // =====================================================================
        void TeleopDirectionController(Keyboard kb)
        {
            // digit4 exits back to ManualController
            if (kb.digit4.wasPressedThisFrame)
            {
                control.CancelActiveAction();
                control.controlRights = true;
                _currentMode = ControlMode.ManualController;
                Console.WriteLine("Exited Teleop Mode. Returning to Initial Position.");
                control.ExecuteInitialPosition(control.resetDir, 20f);
                return;
            }

            int direction = 0;
            if      (kb.w.wasPressedThisFrame) direction = 1;
            else if (kb.d.wasPressedThisFrame) direction = 2;
            else if (kb.s.wasPressedThisFrame) direction = 3;
            else if (kb.a.wasPressedThisFrame) direction = 4;

            if (direction != 0)
            {
                control.resetDir = (direction + 2) % 4 + 1;
                Console.WriteLine($"Direction {direction} chosen. Transitioning to Teleoperation pose.");
                _currentMode = ControlMode.TeleopHandControl;
                control.ExecuteTeleoperationPosition(direction, 30f);
            }
        }

        // =====================================================================
        //  Mode 3 — Teleop hand control
        // =====================================================================
        void TeleopHandController(Keyboard kb)
        {
            // digit4 exits back to ManualController
            if (kb.digit4.wasPressedThisFrame)
            {
                control.CancelActiveAction();
                control.controlRights = true;
                _currentMode = ControlMode.ManualController;
                Console.WriteLine("Exited Teleop Mode. Returning to Initial Position.");
                control.ExecuteInitialPosition(control.resetDir, 20f);
                return;
            }

            if (kb.q.isPressed) control.Articulate(0, true);
            if (kb.w.isPressed) control.Articulate(0, false);
            if (kb.a.isPressed) control.Articulate(1, true);
            if (kb.z.isPressed) control.Articulate(1, false);
            if (kb.s.isPressed) control.Articulate(2, true);
            if (kb.x.isPressed) control.Articulate(2, false);
            if (kb.space.wasPressedThisFrame) { _ = control.Grab(); }
        }

        public List<sbyte> GetStateAngles() => stateAngles;
        public sbyte GetStateAngle(int index) => controlAngles[index];
        public void SetControlAngles(List<sbyte> newList) => controlAngles = newList;
        public void SetControlAngle(sbyte value, int index) => controlAngles[index] = value;
    }

    // =========================================================================
    //  Control
    // =========================================================================
    public class Control
    {
        public int   rotLook  = 30;
        internal int _rotLook = 0;
        public float rotStep  = 50f;

        internal int initialStandingAngle = 25;
        private  int stepAngle = 30;

        public GaitType selectedGait = GaitType.Trot;

        private Mover[] _rightForward;
        private Mover[] _rightHind;
        private Mover[] _leftHind;
        private Mover[] _leftForward;

        public bool controlRights = true;

        internal int L_UA = 155;
        internal int L_FA = 275;
        internal int L_Sh = 95;

        private int speed = 25;
        internal int resetDir = 1;

        private CancellationTokenSource? _activeActionCts;

        private const int TickHz = 50;
        private static readonly TimeSpan Tick = TimeSpan.FromMilliseconds(1000.0 / TickHz);

        public Control()
        {
            _rightForward = MakeLeg(0);
            _rightHind    = MakeLeg(1);
            _leftHind     = MakeLeg(2);
            _leftForward  = MakeLeg(3);
            controlRights = true;
        }

        private static Mover[] MakeLeg(int limb) => new[]
        {
            new Mover(-30,  30, limb),
            new Mover(-90,  30, limb + 4),
            new Mover(-90,  90, limb + 8)
        };

        // =====================================================================
        //  Action wrappers
        // =====================================================================
        public void CancelActiveAction() => _activeActionCts?.Cancel();

        public void ExecuteInitialPosition(int direction, float timeout)
            => ExecuteAction(ct => InitialPosition(direction, speed, ct), timeout);

        public void ExecuteStepAction(int direction, float timeout)
            => ExecuteAction(ct => ExecuteStep(direction, speed, ct), timeout);

        public void ExecuteRotateBase(float rotation, float timeout)
            => ExecuteAction(ct => RotateBase(rotation, speed, ct), timeout);

        public void ExecuteRotatePosition(float angle, float timeout)
            => ExecuteAction(ct => RotatePosition(angle, speed, ct), timeout);

        public void ExecuteTeleoperationPosition(int direction, float timeout)
            => ExecuteAction(ct => TeleoperationPosition(direction, speed, 0, ct), timeout);

        public void ExecuteSitDown(float timeout)
            => ExecuteAction(ct => SitDown(speed, ct), timeout);

        public void ExecuteStepAngle(int direction, int angle)
            => ExecuteAction(ct => ExecuteAngle(direction, angle, speed, ct), 12f);

        public void ExecuteCMD(string cmd)
        {
            var command = cmd.Split("_");
            int value = 30;
            if (command.Length > 1) { value = int.Parse(command[1]); Console.WriteLine(command[0] + " " + command[1]); }
            else command[0] = cmd;

            switch (command[0])
            {
                case "initial":      ExecuteInitialPosition(resetDir, 20f); break;
                case "forward":      ExecuteStepAction(1, 12f); break;
                case "right":        ExecuteStepAction(2, 12f); break;
                case "backward":     ExecuteStepAction(3, 12f); break;
                case "left":         ExecuteStepAction(4, 12f); break;
                case "forwardKMPC":  MoveRobotByKMPC(1, value); break;
                case "rightKMPC":    MoveRobotByKMPC(2, value); break;
                case "backwardKMPC": MoveRobotByKMPC(3, value); break;
                case "leftKMPC":     MoveRobotByKMPC(4, value); break;
                case "forwardAuto":  ExecuteStepAngle(1, value); break; ///NEW
                case "rightAuto":    ExecuteStepAngle(2, value); break; ///NEW
                case "backwardAuto": ExecuteStepAngle(3, value); break; ///NEW
                case "leftAuto":     ExecuteStepAngle(4, value); break; ///NEW
                case "sitdown":      ExecuteSitDown(20f); break;
                default: Console.WriteLine($"Unknown command: {cmd}"); break;
            }
        }

        // =====================================================================
        //  Timeout & cancellation
        // =====================================================================
        private async void ExecuteAction(Func<CancellationToken, Task> action, float timeoutSeconds)
        {
            if (!controlRights) return;
            controlRights = false;

            _activeActionCts?.Cancel();
            _activeActionCts?.Dispose();
            _activeActionCts = new CancellationTokenSource();
            _ = SafeCancelAfterDelay(_activeActionCts, timeoutSeconds);

            try   { await action(_activeActionCts.Token); }
            catch (OperationCanceledException) { Console.WriteLine($"Action timed out after {timeoutSeconds}s."); }
            catch (Exception ex) { Console.Error.WriteLine($"Action failed: {ex.Message}"); }
            finally { controlRights = true; }
        }

        private async Task SafeCancelAfterDelay(CancellationTokenSource cts, float timeoutSeconds)
        {
            try
            {
                await Task.Delay(TimeSpan.FromSeconds(timeoutSeconds), cts.Token);
                if (!cts.IsCancellationRequested) cts.Cancel();
            }
            catch (OperationCanceledException) { }
            catch (ObjectDisposedException) { }
        }

        // =====================================================================
        //  Leg helpers
        // =====================================================================
        private async Task RotateLegAsync(Mover[] leg, float yaw, float shoulder, float elbow, float spd, CancellationToken ct)
        {
            await Task.WhenAll(
                leg[0].RotateTarget(yaw,      spd, ct),
                leg[1].RotateTarget(shoulder, spd, ct),
                leg[2].RotateTarget(elbow,    spd, ct));
        }

        private void RotateLeg(Mover[] leg, float yaw, float shoulder, float elbow, float spd, CancellationToken ct)
        {
            _ = leg[0].RotateTarget(yaw,      spd, ct);
            _ = leg[1].RotateTarget(shoulder, spd, ct);
            _ = leg[2].RotateTarget(elbow,    spd, ct);
        }

        // =====================================================================
        //  Directional gait controller
        // =====================================================================
        private async Task ExecuteStep(int direction, float speedRotation, CancellationToken ct)
        {
            Mover[][] movers = { _rightForward, _rightHind, _leftHind, _leftForward };
            int dirIndex = direction - 1;

            Mover[] frontRight = movers[dirIndex];
            Mover[] backRight  = movers[(dirIndex + 1) % 4];
            Mover[] backLeft   = movers[(dirIndex + 2) % 4];
            Mover[] frontLeft  = movers[(dirIndex + 3) % 4];

            switch (selectedGait)
            {
                case GaitType.Trot:
                    speedRotation*=5f;
                    if (Math.Abs(frontRight[0].Target()) > 3f ||
                        Math.Abs(backLeft[0].Target())   > 3f ||
                        Math.Abs(backRight[1].Target() + initialStandingAngle) > 3f ||
                        Math.Abs(frontLeft[1].Target() + initialStandingAngle) > 3f ||
                        Math.Abs(backRight[2].Target() + 90 - initialStandingAngle) > 3f ||
                        Math.Abs(frontLeft[2].Target() + 90 - initialStandingAngle) > 3f ||
                        Math.Abs(frontRight[1].Target() + initialStandingAngle) > 3f ||
                        Math.Abs(backLeft[1].Target() + initialStandingAngle)   > 3f ||
                        Math.Abs(frontRight[2].Target() + 90 - initialStandingAngle) > 3f ||
                        Math.Abs(backLeft[2].Target() + 90 - initialStandingAngle)   > 3f)
                    {
                        await InitialPosition(resetDir, speedRotation, ct);
                    }
                    resetDir = direction;
                    await TrotGait(frontRight, backRight, backLeft, frontLeft, speedRotation / 2, ct);
                    break;

                case GaitType.Canter:
                    speedRotation*=1.25f;
                    if (Math.Abs(frontRight[0].Target() ) > 3f ||
                        Math.Abs(backRight[0].Target()  ) > 3f ||
                        Math.Abs(backLeft[0].Target()  ) > 3f ||
                        Math.Abs(frontLeft[0].Target()  ) > 3f)
                    {
                        await InitialPosition(resetDir, speedRotation, ct);
                    }
                    resetDir = direction;
                    await CanterGait(frontRight, backRight, backLeft, frontLeft, speedRotation, ct);
                    break;

                case GaitType.Gallop:
                    speedRotation*=1.75f;
                    if (Math.Abs(frontRight[0].Target()  ) > 3f ||
                        Math.Abs(backLeft[0].Target()  ) > 3f)
                    {
                        await InitialPosition(resetDir, speedRotation, ct);
                    }
                    resetDir = direction+1;
                    if (resetDir>4) resetDir=1;
                    await GallopGait(frontRight, backRight, backLeft, frontLeft, speedRotation, ct);
                    break;
            }
        }




        private async Task ExecuteAngle(int direction, int _walkAngle, float speedRotation, CancellationToken ct)
        {
            Mover[][] movers = { _rightForward, _rightHind, _leftHind, _leftForward };
            int dirIndex = direction - 1;

            Mover[] frontRight = movers[dirIndex];
            Mover[] backRight  = movers[(dirIndex + 1) % 4];
            Mover[] backLeft   = movers[(dirIndex + 2) % 4];
            Mover[] frontLeft  = movers[(dirIndex + 3) % 4];

            switch (selectedGait)
            {
                case GaitType.Trot:
                    speedRotation*=5f;
                    _walkAngle=(int)Clamp(_walkAngle,-15f,15f);
                    resetDir = direction;
                    await TrotGaitAngled(frontRight, backRight, backLeft, frontLeft, _walkAngle, speedRotation / 2, ct);
                    break;

                case GaitType.Canter:
                    speedRotation*=1.25f;
                    _walkAngle=(int)Clamp(_walkAngle,-15f,15f);
                    resetDir = direction;
                    await CanterGaitAngled(frontRight, backRight, backLeft,  frontLeft, _walkAngle,  speedRotation, ct);
                    break;

                case GaitType.Gallop:
                    speedRotation*=1.75f;
                    resetDir = direction+1;
                    if (resetDir>4) resetDir=1;
                    _walkAngle=(int)Clamp(_walkAngle,-15f,15f);
                    await GallopGaitAngled(frontRight, backRight, backLeft,   frontLeft, _walkAngle, speedRotation, ct);
                    break;
            }
        }

        // =====================================================================
        //  Gaits
        // =====================================================================
        private async Task TrotGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
        {
            if (Math.Abs(bl[0].CurrentPrimaryAxisRotation()) > 0.5f)
                await RotateLegAsync(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            if (Math.Abs(fr[0].CurrentPrimaryAxisRotation()) > 0.5f)
                await RotateLegAsync(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

            _ = Halfstep(br, true,  speedRotation, 0, ct);
            await Halfstep(fl, false, speedRotation, 0, ct);
            await Stepper(br, fl, bl, fr, speedRotation, ct);
        }


        private async Task TrotGaitAngled(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, int _walkAngle, float speedRotation, CancellationToken ct)
        {
            // if (Math.Abs(bl[0].CurrentPrimaryAxisRotation()-_walkAngle) > 3f)
            //     await RotateLegAsync(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            // if (Math.Abs(fr[0].CurrentPrimaryAxisRotation()-_walkAngle) > 0.5f)
            //     await RotateLegAsync(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            _walkAngle = (int)Clamp(_walkAngle, -15f, 15f);
            _=fr[0].RotateTarget(0, speedRotation, ct);
            _=bl[0].RotateTarget(0, speedRotation, ct);
            _ = HalfstepAngled(br, true, _walkAngle,  speedRotation, 0, ct);
            await HalfstepAngled(fl, false, _walkAngle, speedRotation, 0, ct);
            await StepperAngled(br, fl, bl, fr, _walkAngle, speedRotation, ct);
        }

        private async Task CanterGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotat, CancellationToken ct)
        {
            int N = 25, total = 50;
            float[][] traj   = trajectoryGenerator(N, 0.9f, 0.5f);
            float[][] angles = inverseKinematics(traj[0], traj[1]);
            int start = N / 2;
            float spd = 50f;

            await Task.WhenAll(
                fr[1].RotateTarget(angles[0][start], spd, ct),
                fr[2].RotateTarget(angles[1][start], spd, ct),
                fl[1].RotateTarget(angles[0][(start + N) % total], spd, ct),
                fl[2].RotateTarget(angles[1][(start + N) % total], spd, ct),
                br[1].RotateTarget(angles[0][total - 1 - start], spd, ct),
                br[2].RotateTarget(angles[1][total - 1 - start], spd, ct),
                bl[1].RotateTarget(angles[0][(3 * N - start) % total], spd, ct),
                bl[2].RotateTarget(angles[1][(3 * N - start) % total], spd, ct));

            spd = speedRotat * 5;
            for (int step = 0; step < total && !ct.IsCancellationRequested; step++)
            {
                int i = (start + step) % total;
                await Task.WhenAll(
                    fr[1].RotateTarget(angles[0][i], spd, ct),
                    fr[2].RotateTarget(angles[1][i], spd, ct),
                    fl[1].RotateTarget(angles[0][(i + N) % total], spd, ct),
                    fl[2].RotateTarget(angles[1][(i + N) % total], spd, ct),
                    br[1].RotateTarget(angles[0][total - 1 - i], spd, ct),
                    br[2].RotateTarget(angles[1][total - 1 - i], spd, ct),
                    bl[1].RotateTarget(angles[0][(3 * N - i) % total], spd, ct),
                    bl[2].RotateTarget(angles[1][(3 * N - i) % total], spd, ct));
                await Task.Delay(5, ct);
                await Task.Yield();
            }
        }

        private async Task CanterGaitAngled(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, int _walkAngle, float speedRotat, CancellationToken ct)
        {
            int N = 25, total = 50;
            float[][] traj   = trajectoryGenerator(N, 0.9f, 0.5f);
            float[][] angles = inverseKinematics(traj[0], traj[1]);
            int start = N / 2;
            float spd = 50f;

            await Task.WhenAll(
                fr[1].RotateTarget(angles[0][start], spd, ct),
                fr[2].RotateTarget(angles[1][start], spd, ct),
                fl[1].RotateTarget(angles[0][(start + N) % total], spd, ct),
                fl[2].RotateTarget(angles[1][(start + N) % total], spd, ct),
                
                br[1].RotateTarget(angles[0][total - 1 - start], spd, ct),
                br[2].RotateTarget(angles[1][total - 1 - start], spd, ct),

                bl[1].RotateTarget(angles[0][(3 * N - start) % total], spd, ct),
                bl[2].RotateTarget(angles[1][(3 * N - start) % total], spd, ct));

            spd = speedRotat * 5;
            for (int step = 0; step < total && !ct.IsCancellationRequested; step++)
            {
                int i = (start + step) % total;
                await Task.WhenAll(
                    fr[1].RotateTarget(angles[0][i], spd, ct),
                    fr[2].RotateTarget(angles[1][i], spd, ct),
                    fl[1].RotateTarget(angles[0][(i + N) % total], spd, ct),
                    fl[2].RotateTarget(angles[1][(i + N) % total], spd, ct),
                    br[1].RotateTarget(angles[0][total - 1 - i], spd, ct),
                    br[2].RotateTarget(angles[1][total - 1 - i], spd, ct),
                    bl[1].RotateTarget(angles[0][(3 * N - i) % total], spd, ct),
                    bl[2].RotateTarget(angles[1][(3 * N - i) % total], spd, ct));

                if (step<total/2)
                    {fl[0].RotateTarget(_walkAngle,spd,ct);
                    br[0].RotateTarget(_walkAngle,spd,ct);
                    fr[0].RotateTarget(0,spd,ct);
                    bl[0].RotateTarget(0,spd,ct);}

                if (step>total/2)
                    {fr[0].RotateTarget(_walkAngle,spd,ct);
                    bl[0].RotateTarget(_walkAngle,spd,ct);
                    fl[0].RotateTarget(0,spd,ct);
                    br[0].RotateTarget(0,spd,ct);}
                await Task.Delay(5, ct);
                await Task.Yield();
            }
        }


        private async Task GallopGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
        {
            (fr, br, bl, fl) = (bl, fl, fr, br);  // original offset swap from MixWalk

            int N = 50, total = 100;
            float[][] traj    = trajectoryGenerator(N, 0.8f, 0.7f);
            float[][] angles  = inverseKinematics(traj[0], traj[1]);
            float[][] traj1   = trajectoryGenerator(N, 0.8f, 0.7f);
            float[][] angles1 = inverseKinematics(traj1[0], traj1[1]);

            float targetAngle    = 30f;
            float rotationChange = 2 * targetAngle / N;
            float rotation       = 0f;
            float l_max          = L_UA * MathF.Cos(initialStandingAngle * MathF.PI / 180) + L_Sh;
            float startAngle     = 0f;

            await Task.WhenAll(
                bl[1].RotateTarget(angles1[0][0], speedRotation, ct),
                bl[2].RotateTarget(angles1[1][0], speedRotation, ct),
                fr[1].RotateTarget(angles[0][(3 * N) % total], speedRotation, ct),
                fr[2].RotateTarget(angles[1][(3 * N) % total], speedRotation, ct),
                br[1].RotateTarget(-initialStandingAngle, speedRotation, ct),
                br[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct),
                fl[1].RotateTarget(-initialStandingAngle, speedRotation, ct),
                fl[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct));

            await Task.Delay((int)(1000 / speedRotation), ct);

            for (int i = 0; i < total && !ct.IsCancellationRequested; i++)
            {
                bl[1].RotateTo((sbyte)angles1[0][i]);
                bl[2].RotateTo((sbyte)angles1[1][i]);
                fr[1].RotateTo((sbyte)angles[0][(3 * N - i) % total]);
                fr[2].RotateTo((sbyte)angles[1][(3 * N - i) % total]);

                if (i == N / 4 - 1) startAngle = fl[0].CurrentPrimaryAxisRotation();

                if (i <= N / 4)
                {
                    float t0 = Lerp(-initialStandingAngle, 0, 4f * i / N);
                    float t1 = Lerp(-90 + initialStandingAngle, -90, 4f * i / N);
                    rotation = 0f;
                    br[1].RotateTo((sbyte)t0); br[2].RotateTo((sbyte)t1);
                    fl[1].RotateTo((sbyte)t0); fl[2].RotateTo((sbyte)t1);
                }
                else if (i <= N * 3 / 4)
                {
                    float t2 = Lerp(startAngle, targetAngle, 2f * i / N - 0.5f);
                    fl[0].RotateTo((sbyte)t2);
                    br[0].RotateTo((sbyte)(-t2));
                }
                else if (i <= N)
                {
                    float t3 = Lerp(0, -initialStandingAngle, 4f * i / N - 3f);
                    float t4 = Lerp(-90, -90 + initialStandingAngle, 4f * i / N - 3f);
                    br[1].RotateTo((sbyte)t3); br[2].RotateTo((sbyte)t4);
                    fl[1].RotateTo((sbyte)t3); fl[2].RotateTo((sbyte)t4);
                }
                else
                {
                    float rotGoalLeft  = br[0].CurrentPrimaryAxisRotation() + rotationChange;
                    float rotGoalRight = fl[0].CurrentPrimaryAxisRotation() - rotationChange;
                    rotation += -MathF.Sign(br[0].CurrentPrimaryAxisRotation()) * rotationChange;
                    float gh = MathF.Acos(
                        2 * (MathF.Cos(rotation * MathF.PI / 180) - 1) * l_max / L_UA
                        + MathF.Cos(initialStandingAngle * MathF.PI / 180)) * 180 / MathF.PI - initialStandingAngle;

                    fl[0].RotateTo((sbyte)rotGoalRight); br[0].RotateTo((sbyte)rotGoalLeft);
                    fl[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                    fl[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));
                    br[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                    br[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));
                }

                await Task.Delay((int)(1000 / (1.5f * speedRotation)), ct);
                await Task.Yield();
            }
        }


        private async Task GallopGaitAngled(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, int _walkAngle, float speedRotation, CancellationToken ct)
        {
            (fr, br, bl, fl) = (bl, fl, fr, br);  // original offset swap from MixWalk

            int N = 50, total = 100;
            float[][] traj    = trajectoryGenerator(N, 0.8f, 0.7f);
            float[][] angles  = inverseKinematics(traj[0], traj[1]);
            float[][] traj1   = trajectoryGenerator(N, 0.8f, 0.7f);
            float[][] angles1 = inverseKinematics(traj1[0], traj1[1]);

            float targetAngle    = 30f;
            float rotationChange = 2 * targetAngle / N;
            float rotation       = 0f;
            float l_max          = L_UA * MathF.Cos(initialStandingAngle * MathF.PI / 180) + L_Sh;
            float startAngle     = 0f;

            float startAngle2 = 0f;

            await Task.WhenAll(
                bl[1].RotateTarget(angles1[0][0], speedRotation, ct),
                bl[2].RotateTarget(angles1[1][0], speedRotation, ct),
                fr[1].RotateTarget(angles[0][(3 * N) % total], speedRotation, ct),
                fr[2].RotateTarget(angles[1][(3 * N) % total], speedRotation, ct),
                br[1].RotateTarget(-initialStandingAngle, speedRotation, ct),
                br[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct),
                fl[1].RotateTarget(-initialStandingAngle, speedRotation, ct),
                fl[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct));

            await Task.Delay((int)(1000 / speedRotation), ct);

            for (int i = 0; i < total && !ct.IsCancellationRequested; i++)
            {
                bl[1].RotateTo((sbyte)angles1[0][i]);
                bl[2].RotateTo((sbyte)angles1[1][i]);
                fr[1].RotateTo((sbyte)angles[0][(3 * N - i) % total]);
                fr[2].RotateTo((sbyte)angles[1][(3 * N - i) % total]);

                if (i == N / 4 - 1) 
                {
                    startAngle = fl[0].CurrentPrimaryAxisRotation();
                    startAngle2=br[0].CurrentPrimaryAxisRotation();
                }

                if (i <= N / 4)
                {
                    float t0 = Lerp(-initialStandingAngle, 0, 4f * i / N);
                    float t1 = Lerp(-90 + initialStandingAngle, -90, 4f * i / N);
                    rotation = 0f;
                    br[1].RotateTo((sbyte)t0); br[2].RotateTo((sbyte)t1);
                    fl[1].RotateTo((sbyte)t0); fl[2].RotateTo((sbyte)t1);

                }
                else if (i <= N * 3 / 4)
                {   
                    if (_walkAngle>3 || _walkAngle < -3f)
                    {
                        var ang1=Lerp(startAngle, _walkAngle+15, 2f * i / N - 0.5f);
                        var ang2=Lerp(startAngle2, _walkAngle-15, 2f * i / N - 0.5f);

                        fl[0].RotateTo((sbyte)ang1);
                        br[0].RotateTo((sbyte)ang2);
                    }
                    else
                    {
                        float t2 = Lerp(startAngle, targetAngle, 2f * i / N - 0.5f);
                        fl[0].RotateTo((sbyte)t2);
                        br[0].RotateTo((sbyte)(-t2));
                    }
                    bl[0].RotateTarget(0,speedRotation,ct);
                    fr[0].RotateTarget(0,speedRotation,ct);
                }
                else if (i <= N)
                {
                    float t3 = Lerp(0, -initialStandingAngle, 4f * i / N - 3f);
                    float t4 = Lerp(-90, -90 + initialStandingAngle, 4f * i / N - 3f);
                    br[1].RotateTo((sbyte)t3); br[2].RotateTo((sbyte)t4);
                    fl[1].RotateTo((sbyte)t3); fl[2].RotateTo((sbyte)t4);
                }
                else
                {
                     if (_walkAngle>3 || _walkAngle < -3f)  rotationChange = 2f * (15f) / N;
                    float rotGoalLeft  = br[0].CurrentPrimaryAxisRotation() + rotationChange;
                    float rotGoalRight = fl[0].CurrentPrimaryAxisRotation() - rotationChange;
                    rotation += -MathF.Sign(br[0].CurrentPrimaryAxisRotation()) * rotationChange;
                    float gh = MathF.Acos(
                        2 * (MathF.Cos(rotation * MathF.PI / 180) - 1) * l_max / L_UA
                        + MathF.Cos(initialStandingAngle * MathF.PI / 180)) * 180 / MathF.PI - initialStandingAngle;

                    fl[0].RotateTo((sbyte)rotGoalRight); br[0].RotateTo((sbyte)rotGoalLeft);
                    fl[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                    fl[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));
                    br[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                    br[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));

                    bl[0].RotateTarget(_walkAngle,speedRotation,ct);
                    fr[0].RotateTarget(_walkAngle,speedRotation,ct);
                }

                await Task.Delay((int)(1000 / (1.5f * speedRotation)), ct);
                await Task.Yield();
            }
        }

        private async Task Stepper(Mover[] right, Mover[] left, Mover[] back, Mover[] forward, float speedRotation, CancellationToken ct)
        {
            _ = back; _ = forward;
            float targetAngle          = stepAngle;
            float initialRotationRight = right[0].CurrentPrimaryAxisRotation();
            float initialRotationLeft  = left[0].CurrentPrimaryAxisRotation();
            float l_max        = L_UA * MathF.Cos(initialStandingAngle * MathF.PI / 180) + L_Sh;
            float progress     = 0f;
            float totalDuration = (targetAngle * 2) / speedRotation;
            const float dt = 1f / TickHz;

            while (progress < totalDuration && !ct.IsCancellationRequested)
            {
                progress += dt;
                float rotTotal = (progress / totalDuration) * (2 * targetAngle);
                float rot = rotTotal < targetAngle ? rotTotal : 2 * targetAngle - rotTotal;
                float gh = MathF.Acos(
                    2 * (MathF.Cos(rot * MathF.PI / 180) - 1) * l_max / L_UA
                    + MathF.Cos(initialStandingAngle * MathF.PI / 180)) * 180 / MathF.PI - initialStandingAngle;

                float gLeft  = initialRotationLeft  + rotTotal;
                float gRight = initialRotationRight - rotTotal;

                right[0].RotateTo((sbyte)gRight);
                left[0].RotateTo((sbyte)gLeft);
                right[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                right[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));
                left[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                left[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));

                await Task.Delay(Tick, ct);
            }
        }


        private async Task Halfstep(Mover[] limb, bool side, float speedRotation, float grabAngle, CancellationToken ct)
        {
            _ = limb[1].RotateTarget(0, speedRotation, ct);
            await limb[2].RotateTarget(-90, speedRotation, ct);
            await limb[0].RotateTarget(side ? stepAngle : -stepAngle, speedRotation, ct);
            _ = limb[1].RotateTarget(-initialStandingAngle + grabAngle, speedRotation, ct);
            await limb[2].RotateTarget(-90 + initialStandingAngle - grabAngle, speedRotation, ct);
        }

        private async Task HalfstepAngled(Mover[] limb, bool side, int _walkAngle, float speedRotation, float grabAngle, CancellationToken ct)
        {
            float opt1;
            float opt2;

            if (_walkAngle==0)
            {
                opt1 = stepAngle;
                opt2 = -stepAngle;
            }
            else
            {
                opt1 = _walkAngle + stepAngle/2;
                opt2 = _walkAngle - stepAngle/2;
            }
            _ = limb[1].RotateTarget(0, speedRotation, ct);
            await limb[2].RotateTarget(-90, speedRotation, ct);
            await limb[0].RotateTarget(side ? (opt1) : (opt2), speedRotation, ct);
            _ = limb[1].RotateTarget(-initialStandingAngle + grabAngle, speedRotation, ct);
            await limb[2].RotateTarget(-90 + initialStandingAngle - grabAngle, speedRotation, ct);
        }

        private async Task StepperAngled(Mover[] right, Mover[] left, Mover[] back, Mover[] forward, int _walkAngle, float speedRotation, CancellationToken ct)
        {
            _ = back; _ = forward;
            float targetAngle          = stepAngle/2;

            if (_walkAngle==0) targetAngle*=2;

            float initialRotationRight = right[0].CurrentPrimaryAxisRotation();
            float initialRotationLeft  = left[0].CurrentPrimaryAxisRotation();
            float l_max        = L_UA * MathF.Cos(initialStandingAngle * MathF.PI / 180) + L_Sh;
            float progress     = 0f;
            float totalDuration = (targetAngle * 2) / speedRotation;
            const float dt = 1f / TickHz;


            float initialRotationBack = back[0].CurrentPrimaryAxisRotation();
            float initialRotationForward = forward[0].CurrentPrimaryAxisRotation();


            while (progress < totalDuration && !ct.IsCancellationRequested)
            {
                progress += dt;
                float rotTotal = (progress / totalDuration) * (2 * targetAngle);
                float rot = rotTotal < targetAngle ? rotTotal : 2 * targetAngle - rotTotal;
                float gh = MathF.Acos(
                    2 * (MathF.Cos(rot * MathF.PI / 180) - 1) * l_max / L_UA
                    + MathF.Cos(initialStandingAngle * MathF.PI / 180)) * 180 / MathF.PI - initialStandingAngle;

                float gLeft  = initialRotationLeft  + rotTotal;
                float gRight = initialRotationRight - rotTotal;

                // float gFwd = initialRotationForward + (progress / totalDuration) * (_walkAngle);

                right[0].RotateTo((sbyte)gRight);
                left[0].RotateTo((sbyte)gLeft);
                right[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                right[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));
                left[1].RotateTo((sbyte)(-initialStandingAngle - gh));
                left[2].RotateTo((sbyte)(-90 + initialStandingAngle + gh));

                if (progress>totalDuration/3f)
                { back[0].RotateTarget((float)_walkAngle,speedRotation,ct);
                forward[0].RotateTarget((float)_walkAngle,speedRotation,ct);}
                // forward[0].RotateTo((sbyte)gFwd);
                back[1].RotateTo((sbyte)(-initialStandingAngle + gh/2));
                back[2].RotateTo((sbyte)(-90 + initialStandingAngle - gh/2));
                forward[1].RotateTo((sbyte)(-initialStandingAngle + gh/2));
                forward[2].RotateTo((sbyte)(-90 + initialStandingAngle - gh/2));


                await Task.Delay(Tick, ct);
            }
        }

        // =====================================================================
        //  Core positioning
        // =====================================================================
        internal async Task InitialPositionDirectional(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
        {
            speedRotation = 75f;

            async Task ResetLeg(Mover[] leg)
            {
                if (Math.Abs(leg[0].CurrentPrimaryAxisRotation()) > 3)
                {
                    await RotateLegAsync(leg, leg[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
                    await RotateLegAsync(leg, 0, 0, -90, speedRotation, ct);
                }
                else if (Math.Abs(leg[1].CurrentPrimaryAxisRotation() + initialStandingAngle) > 3f ||
                         Math.Abs(leg[2].CurrentPrimaryAxisRotation() + 90 - initialStandingAngle) > 3f)
                {
                    await RotateLegAsync(leg, 0, 0, -90, speedRotation, ct);
                }
                await RotateLegAsync(leg, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            }

            await ResetLeg(br);
            await ResetLeg(fl);
            await ResetLeg(fr);
            await ResetLeg(bl);
            await Task.Yield();
        }

        internal async Task InitialPosition(int direction, float speedRotation, CancellationToken ct)
        {
            Mover[][] movers = { _rightForward, _rightHind, _leftHind, _leftForward };
            int d = direction - 1;
            await InitialPositionDirectional(
                movers[d], movers[(d + 1) % 4], movers[(d + 2) % 4], movers[(d + 3) % 4],
                speedRotation, ct);
        }

        public async Task RotateBase(float rotation, float speedRotation, CancellationToken ct)
        {
            speedRotation = 50;
            float a = _rightForward[0].Target(), b = _rightHind[0].Target(),
                  c = _leftForward[0].Target(),  d = _leftHind[0].Target();
            if (a != b || a != c || a != d) await InitialPosition(resetDir, speedRotation, ct);

            float rotAngle = Clamp(rotation, -30, 30);
            RotateLeg(_rightForward, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            RotateLeg(_rightHind,    rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            RotateLeg(_leftForward,  rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            await RotateLegAsync(_leftHind, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            await Task.Delay(1000, ct);
        }

        public async Task RotatePosition(float angle, float speedRotation, CancellationToken ct)
        {
            speedRotation = 75f;
            angle = Clamp(angle, -59, 59);
            if (Math.Abs(angle) <= 30)
            {
                await RotateBase(angle, speedRotation, ct);
                await InitialPosition(resetDir, speedRotation, ct);
            }
            else
            {
                await RotateBase(-MathF.Sign(angle) * 30, speedRotation, ct);
                await Restep(angle % 30, speedRotation, ct);
                await RotateBase(0, speedRotation, ct);
            }
        }

        internal async Task Restep(float angle, float speedRotation, CancellationToken ct)
        {
            Mover[] fr = _rightForward, br = _rightHind, bl = _leftHind, fl = _leftForward;
            foreach (var leg in new[] { br, fl, fr, bl })
            {
                await RotateLegAsync(leg, leg[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
                await RotateLegAsync(leg, angle, 0, -90, speedRotation, ct);
                await RotateLegAsync(leg, angle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
            }
            await Task.Yield();
        }

        // =====================================================================
        //  Teleoperation
        // =====================================================================
        private int teleopDirection = 1;

        public async Task TeleoperationPosition(int direction, float speedRotation, float grabAngle, CancellationToken ct)
        {
            speedRotation = 100f;
            Mover[][] movers = { _leftHind, _leftForward, _rightForward, _rightHind };
            await InitialPositionDirectional(_rightForward, _rightHind, _leftHind, _leftForward, speedRotation, ct);

            int forward = direction - 1, right = direction % 4,
                back = (direction + 1) % 4, left = (direction + 2) % 4;
            teleopDirection = back;

            await RotateLegAsync(movers[right], movers[right][0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(movers[right], -30, 0, -90, speedRotation, ct);
            await RotateLegAsync(movers[right], -30, -initialStandingAngle + grabAngle, -90 + initialStandingAngle - grabAngle, speedRotation, ct);

            await RotateLegAsync(movers[left], movers[left][0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(movers[left], 30, 0, -90, speedRotation, ct);
            await RotateLegAsync(movers[left], 30, -initialStandingAngle + grabAngle, -90 + initialStandingAngle - grabAngle, speedRotation, ct);

            var tf = RotateLegAsync(movers[forward], 0, 0, -90, speedRotation, ct);
            var tb = RotateLegAsync(movers[back],    0, 0,   0, speedRotation, ct);
            await Task.WhenAll(tf, tb);
        }

        public void Articulate(int index, bool rot)
        {
            Mover[][] movers = { _leftHind, _leftForward, _rightForward, _rightHind };
            float angle = movers[teleopDirection][index].Target();
            int sign = rot ? 1 : -1;
            float min = index == 0 ? -30f : -90f;
            float max = index == 0 ?  30f :  90f;
            angle = Clamp(angle, min, max);
            movers[teleopDirection][index].RotateTo((sbyte)(angle + sign * 0.1f));
        }

        private bool grab = true;
        public Task Grab()
        {
            // PLACEHOLDER: no gripper in the 12-motor set.
            // Wire to your real gripper action/service:
            //   /gripper/command { bool close }
            Console.WriteLine($"Grab ({(grab ? "close" : "open")}) — NOT WIRED");
            grab = !grab;
            return Task.CompletedTask;
        }

        private async Task SitDown(float speedRotation, CancellationToken ct)
        {
            speedRotation = 100f;
            int N = 100;
            float[][] traj   = trajectorySitDown(N);
            float[][] angles = inverseKinematics(traj[0], traj[1]);

            for (int i = 10; i < N && !ct.IsCancellationRequested; i++)
            {
                await Task.WhenAll(
                    _rightForward[1].RotateTarget(angles[0][i], speedRotation, ct),
                    _rightForward[2].RotateTarget(angles[1][i], speedRotation, ct),
                    _leftForward[1].RotateTarget(angles[0][i], speedRotation, ct),
                    _leftForward[2].RotateTarget(angles[1][i], speedRotation, ct),
                    _rightHind[1].RotateTarget(angles[0][i], speedRotation, ct),
                    _rightHind[2].RotateTarget(angles[1][i], speedRotation, ct),
                    _leftHind[1].RotateTarget(angles[0][i], speedRotation, ct),
                    _leftHind[2].RotateTarget(angles[1][i], speedRotation, ct));
                await Task.Yield();
            }
        }

        // =====================================================================
        //  KMPC
        // =====================================================================


        public async void MoveRobotByKMPC(int direction, int angle)
        {
            var cts = new CancellationTokenSource();
            controlRights = false;
            await Task.Delay(100);

            for (int check = 0; check < 3; check++)
            {
                int idx0 = check == 0 ? direction - 1      : check == 1 ? direction + 4 - 1  : -1 + direction + 8;
                int idx1 = check == 0 ? (-1+direction+2)%4 : check == 1 ? (-1+direction+2)%4+4 : (-1+direction+2)%4+8;
                sbyte expect = check == 0 ? (sbyte)0 : check == 1 ? (sbyte)-initialStandingAngle : (sbyte)(-90 + initialStandingAngle);
                if (CommunicatorDotNet.GetAngleDelegate(idx0) != expect || CommunicatorDotNet.GetAngleDelegate(idx1) != expect)
                    await InitialPosition(resetDir, speed, cts.Token);
            }

            await Task.Delay(100);
            Console.WriteLine("Walking KMPC Gait 1 Trajectory");
            await Task.WhenAny(Gait1Walk(direction, cts.Token, angle), Task.Delay(30000));
            cts.Cancel();
            await Task.Delay(100);
            CommunicatorDotNet.Request_KMPC_Delegate(new RosEmpty());
            Console.WriteLine("KMPC Trajectory executed");
            controlRights = true;
        }

        // public async void MoveRobotBySequence(List<sbyte> angles)
        // {
        //     var cts = new CancellationTokenSource();
        //     controlRights = false;
        //     Console.WriteLine("Walking KMPC Trajectories");
        //     await Task.Delay(100);

        //     if (CommunicatorDotNet.GetAngleDelegate(0) != 0 || CommunicatorDotNet.GetAngleDelegate(1) != 0 ||
        //         CommunicatorDotNet.GetAngleDelegate(2) != 0 || CommunicatorDotNet.GetAngleDelegate(3) != 0)
        //         await InitialPosition(resetDir, speed, cts.Token);

        //     await Task.WhenAny(ExecuteTrajectory(angles, cts.Token), Task.Delay(60000));
        //     cts.Cancel();
        //     await Task.Delay(100);
        //     CommunicatorDotNet.Request_KMPC_Delegate(new RosEmpty());
        //     Console.WriteLine("KMPC Trajectory executed");
        //     controlRights = true;
        // }

        // private async Task ExecuteTrajectory(List<sbyte> angles, CancellationToken token)
        // {
        //     for (int i = 0; i < 200; i++)
        //     {
        //         if (token.IsCancellationRequested) return;
        //         _rightForward[0].RotateTo(0); _rightHind[0].RotateTo(0);
        //         _leftHind[0].RotateTo(0);     _leftForward[0].RotateTo(0);

        //         _rightForward[1].RotateTo(angles[i]);
        //         _rightForward[2].RotateTo(angles[200 + i]);
        //         _rightHind[1].RotateTo(angles[400 + i]);
        //         _rightHind[2].RotateTo(angles[600 + i]);
        //         _leftHind[1].RotateTo(angles[800 + i]);
        //         _leftHind[2].RotateTo(angles[1000 + i]);
        //         _leftForward[1].RotateTo(angles[1200 + i]);
        //         _leftForward[2].RotateTo(angles[1400 + i]);

        //         await Task.Delay(20, token);
        //         await Task.Yield();
        //     }
        // }


        // Updated C# code for dynamic trajectory execution

        // Updated MoveRobotBySequence with dynamic size parameter:
        public async void MoveRobotBySequence(List<sbyte> angles, int numPoints)
        {
            var cts = new CancellationTokenSource();
            controlRights = false;
            Console.WriteLine($"Walking KMPC Trajectories ({numPoints} points per leg)");
            await Task.Delay(100);

            if (CommunicatorDotNet.GetAngleDelegate(0) != 0 || CommunicatorDotNet.GetAngleDelegate(1) != 0 ||
                CommunicatorDotNet.GetAngleDelegate(2) != 0 || CommunicatorDotNet.GetAngleDelegate(3) != 0)
                await InitialPosition(resetDir, speed, cts.Token);

            await Task.WhenAny(ExecuteTrajectory(angles, numPoints, cts.Token), Task.Delay(60000));
            cts.Cancel();
            await Task.Delay(10);
            CommunicatorDotNet.Request_KMPC_Delegate(new RosEmpty());
            Console.WriteLine("KMPC Trajectory executed");
            controlRights = true;
        }

        // Updated ExecuteTrajectory with dynamic array indexing:
        private async Task ExecuteTrajectory(List<sbyte> angles, int numPoints, CancellationToken token)
        {
            // Validate array size - should be 8 legs * 2 angles * numPoints
            int expectedSize = 8 * numPoints;
            if (angles.Count < expectedSize)
            {
                Console.WriteLine($"Error: Received {angles.Count} angles, expected at least {expectedSize}");
                return;
            }

            for (int i = 0; i < numPoints; i++)
            {
                if (token.IsCancellationRequested) return;

                // Set shoulder angles to 0
                _rightForward[0].RotateTo(0); 
                _rightHind[0].RotateTo(0);
                _leftHind[0].RotateTo(0);     
                _leftForward[0].RotateTo(0);

                // FR (Front Right) - angles[0 to numPoints-1] and angles[numPoints to 2*numPoints-1]
                _rightForward[1].RotateTo(angles[i]);
                _rightForward[2].RotateTo(angles[numPoints + i]);

                // BR (Back Right) - angles[2*numPoints to 3*numPoints-1] and angles[3*numPoints to 4*numPoints-1]
                _rightHind[1].RotateTo(angles[2 * numPoints + i]);
                _rightHind[2].RotateTo(angles[3 * numPoints + i]);

                // BL (Back Left) - angles[4*numPoints to 5*numPoints-1] and angles[5*numPoints to 6*numPoints-1]
                _leftHind[1].RotateTo(angles[4 * numPoints + i]);
                _leftHind[2].RotateTo(angles[5 * numPoints + i]);

                // FL (Front Left) - angles[6*numPoints to 7*numPoints-1] and angles[7*numPoints to 8*numPoints-1]
                _leftForward[1].RotateTo(angles[6 * numPoints + i]);
                _leftForward[2].RotateTo(angles[7 * numPoints + i]);

                await Task.Delay(10, token);
                await Task.Yield();
            }
        }

        private async Task Gait1Walk(int direction, CancellationToken token, int angle = 30)
        {
            token.ThrowIfCancellationRequested();
            var m = new[] { _rightForward, _rightHind, _leftHind, _leftForward };
            int forward = direction - 1, right = direction % 4,
                back = (direction + 1) % 4, left = (direction + 2) % 4;

            _ = Halfstep(m[right], true, speed, angle, token);
            await Halfstep(m[left], false, speed, angle, token);
            await Task.Delay(10, token);
            await Stepper(m[right], m[left], m[back], m[forward], speed, token);

            m[back][0].RotateTo(0);
            m[back][1].RotateTo((sbyte)-initialStandingAngle);
            await m[back][2].RotateTarget(-90 + initialStandingAngle, speed, token);
            await Task.Delay(10, token);
            m[forward][0].RotateTo(0);
            m[forward][1].RotateTo((sbyte)-initialStandingAngle);
            await m[forward][2].RotateTarget(-90 + initialStandingAngle, speed, token);
            await Task.Delay(10, token);
            await Task.Yield();
        }

        // =====================================================================
        //  Kinematics
        // =====================================================================
        public float[][] trajectoryGenerator(int segment_num, float multiplicator = 0.8f, float archComp = 0.7f)
        {
            float standH  = (float)(Math.Sin(initialStandingAngle * Math.PI / 180) * L_UA + L_FA); //- 10);
            int   shiftFw = (int)(Math.Sqrt((L_UA + L_FA) * (L_UA + L_FA) - standH * standH) * 0.9f);

            var x = new List<float>(); var y = new List<float>();
            for (int i = 0; i < segment_num; i++)
            {
                x.Add((float)(shiftFw - multiplicator * shiftFw * i / segment_num));
                y.Add(-standH);
            }
            for (int j = segment_num; j > 0; j--)
            {
                x.Add((float)(-j / (float)segment_num * multiplicator * shiftFw
                              * Math.Cos(Math.PI - Math.PI * j / segment_num) + shiftFw));
                y.Add((float)(j / (float)segment_num * archComp * shiftFw
                              * Math.Sin(Math.PI - Math.PI * j / segment_num) - standH));

                              
            }
            return new[] { x.ToArray(), y.ToArray() };
        }


        // PLACEHOLDER: replace with the real curve from Calculations.Calculation.
        public float[][] trajectorySitDown(int pointNumber)
        {
            float standH = (float)(Math.Sin(initialStandingAngle * Math.PI / 180) * L_UA + L_FA - 10);
            var x = new List<float>(); var y = new List<float>();
            for (int i = 0; i < pointNumber; i++)
            {
                float t = i / (float)(pointNumber - 1);
                x.Add((float)(t * (L_UA + L_FA) * 0.9));
                y.Add(-standH * (1 - t) - L_FA * 0.1f * t);
            }
            return new[] { x.ToArray(), y.ToArray() };
        }



        public float[][] inverseKinematics(float[] x, float[] y)
        {
            float[][] theta = new[] { new float[x.Length], new float[y.Length] };
            int L1 = L_UA, L2 = L_FA;
            for (int i = 0; i < x.Length; i++)
            {
                float c2 = Clamp((x[i]*x[i] + y[i]*y[i] - L1*L1 - L2*L2) / (2f * L1 * L2), -1f, 1f);
                float s2 = (float)Math.Sqrt(1 - c2 * c2);
                theta[1][i] = (float)(-Math.Acos(c2) * 180 / Math.PI);
                theta[0][i] = (float)((Math.Atan2(y[i], x[i]) + Math.Atan2(L2 * s2, L1 + L2 * c2)) * 180 / Math.PI);
            }
            return theta;
        }

        private static float Clamp(float v, float min, float max) => v < min ? min : (v > max ? max : v);
        private static float Lerp(float a, float b, float t) => a + (b - a) * Clamp(t, 0f, 1f);
    }

    // =========================================================================
    //  Mover — one joint
    // =========================================================================
    public class Mover
    {
        public int lowerLimit;
        public int upperLimit;
        public int rotationState = 0;

        private readonly int index;
        private sbyte _target;

        public Mover(int lowerLimits, int upperLimits, int idx)
        {
            lowerLimit = lowerLimits;
            upperLimit = upperLimits;
            index      = idx;
            _target    = 0;
        }

        /// <summary>Last commanded angle (was articulation.xDrive.target in Unity).</summary>
        public float Target() => _target;

        /// <summary>
        /// Move toward targetAngle at speedRotation deg/s, ticking at 50 Hz.
        /// </summary>
        public async Task RotateTarget(float targetAngle, float speedRotation, CancellationToken token)
        {
            float initial = CurrentPrimaryAxisRotation();
            float delta   = targetAngle - initial;
            if (Math.Abs(delta) < 0.01f) { RotateTo((sbyte)targetAngle); return; }

            rotationState = delta < 0 ? -1 : 1;
            float stepPerTick = Math.Abs(speedRotation) / TickHz;
            float travelled   = 0f;

            while (true)
            {
                if (token.IsCancellationRequested) return;
                if (Math.Abs(delta) - travelled <= stepPerTick)
                {
                    RotateTo((sbyte)(initial + delta));
                    return;
                }
                travelled += stepPerTick;
                float goal = initial + rotationState * travelled;
                if (goal < lowerLimit || goal > upperLimit) return;
                RotateTo((sbyte)goal);
                await Task.Delay(TickInterval, token);
            }
        }

        private const int TickHz = 50;
        private static readonly TimeSpan TickInterval = TimeSpan.FromMilliseconds(1000.0 / TickHz);

        /// <summary>
        /// PLACEHOLDER: returns commanded angle.
        /// For closed-loop, subscribe to /joint_states and map index -> position.
        /// </summary>
        public float CurrentPrimaryAxisRotation() => _target;

        public void RotateTo(sbyte primaryAxisRotation)
        {
            primaryAxisRotation = (sbyte)Math.Clamp((int)primaryAxisRotation, lowerLimit, upperLimit);
            _target = primaryAxisRotation;
            CommunicatorDotNet.SetAngleDelegate(primaryAxisRotation, index);
        }

        // Convenience overload for float callers — clamps and casts internally.
        public void RotateTo(float angle) => RotateTo((sbyte)Math.Clamp((int)angle, lowerLimit, upperLimit));
    }
}
