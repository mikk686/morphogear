using System;
using ROS2;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;
using builtin_interfaces.msg;
using System.Threading;
using System.Diagnostics;
using std_msgs.msg;

using System.IO;
using System.Linq;
using System.Globalization;
using System.Numerics;



namespace ConsoleApplication
{
    public class CommunicatorDotNet
    {
        private static Control control;
        private CommunicatorDotNet()
        {
            RCLdotnet.Init();
            node = RCLdotnet.CreateNode("morphogear");
            control = new Control();


            controlAngles = new List<sbyte> { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            stateAngles = new List<sbyte> { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            SetAngleDelegate = new SetAngle(SetControlAngle);
            GetAngleDelegate = new GetAngle(GetStateAngle);
            Request_KMPC_Delegate = new Request_KMPC(Request_KMPC_Angles);

            Subscription<std_msgs.msg.String> CMDSub = node.CreateSubscription<std_msgs.msg.String>(
                "morphogear_sudo_cmd", CMD_callback);

            Subscription<std_msgs.msg.Bool> ManualSub = node.CreateSubscription<std_msgs.msg.Bool>(
                "morphogear_sudo_manual", Manual_callback);

            Subscription<std_msgs.msg.Int8MultiArray> chatterSub = node.CreateSubscription<std_msgs.msg.Int8MultiArray>(
                "/angles_control", Angles_callback);

            Subscription<std_msgs.msg.Int8MultiArray> KMPCSub = node.CreateSubscription<std_msgs.msg.Int8MultiArray>(
                "/theta_angles", KMPC_callback);



            _KMPCPub = node.CreatePublisher<std_msgs.msg.Empty>("/request_angles");

            _anglePub = node.CreatePublisher<std_msgs.msg.Int8MultiArray>("/angles_control");

            node.CreateTimer(TimeSpan.FromSeconds(0.05), PublishAngles);

        }

        /// <summary>
        /// Angles Properties
        /// </summary>
        private List<sbyte> controlAngles;
        private List<sbyte> stateAngles;

        private bool controlType = false;

        private List<sbyte> KMPCAngles;
        private Subscription<std_msgs.msg.Int8MultiArray> chatterSub;
        private readonly Publisher<std_msgs.msg.Empty> _KMPCPub;
        private readonly Publisher<std_msgs.msg.Int8MultiArray> _anglePub;
        private readonly std_msgs.msg.Int8MultiArray _anglePubMsg = new();

        /// <summary>
        /// Node Properties
        /// </summary>
        private readonly Node node;
        private void Spin() => RCLdotnet.Spin(node);
        public static void Main(string[] args)
        {
            CommunicatorDotNet talker = new CommunicatorDotNet();
            talker.Spin();
        }

        private void Manual_callback(std_msgs.msg.Bool msg)
        {
            controlType = msg.Data;

            if (msg.Data)
            {
                Console.WriteLine("Manual Unity Control");
                node.Timers.Clear();
            }
            else
            {
                Console.WriteLine("Internal Control");
                node.CreateTimer(TimeSpan.FromSeconds(0.05), PublishAngles);
            }
        }

        /// <summary>
        /// Core Manage Functions
        /// </summary>

        private static void CMD_callback(std_msgs.msg.String msg)
        {
            control.ExecuteCMD(msg.Data);
        }

        private void KMPC_callback(std_msgs.msg.Int8MultiArray msg)
        {
            control.MoveRobotBySequence(msg.Data);

        }
        public delegate void Request_KMPC(Empty msg);
        public static Request_KMPC Request_KMPC_Delegate;
        public void Request_KMPC_Angles(Empty msg)
        {
            _KMPCPub.Publish(msg);
        }

        private void Angles_callback(std_msgs.msg.Int8MultiArray msg)
        {
            if (controlType)
                controlAngles = msg.Data;

        }

        private void PublishAngles(TimeSpan elapsed)
        {
            _anglePubMsg.Data = controlAngles;//Console.WriteLine($"Publishing: \"{_anglePubMsg.Data}\"");
            _anglePub.Publish(_anglePubMsg);
        }


        public List<sbyte> GetStateAngles()
        {
            return stateAngles;
        }

        public delegate sbyte GetAngle(int index);
        public static GetAngle GetAngleDelegate;

        public sbyte GetStateAngle(int index)
        {
            return controlAngles[index];
            return stateAngles[index]; // FEEDBACK IF NEEDED
        }



        public void SetControlAngles(List<sbyte> newList)
        {
            controlAngles = newList;
        }

        public delegate void SetAngle(sbyte value, int index);
        public static SetAngle SetAngleDelegate;

        public void SetControlAngle(sbyte value, int index)
        {
            controlAngles[index] = value;
        }

    }



    public class Control
    {
        private static Dictionary<string, Action<int[]>> dict = new Dictionary<string, Action<int[]>>();
        private bool _controlRights = true;

        private Manipulator manipulator= new();
        
        private int initialStandingAngle = 30;

        Mover[,] motors = new Mover[4, 3];

        public Control()
        {
            for (int limb = 0; limb < 4; limb++)
                motors[limb, 0] = new Mover(-30, 30, limb); //Shoulder
            for (int limb = 0; limb < 4; limb++)
                motors[limb, 1] = new Mover(-90, 30, limb + 4); //UpperArm
            for (int limb = 0; limb < 4; limb++)
                motors[limb, 2] = new Mover(-90, 90, limb + 8); //ForeArm



            dict.Add("initial", (x) => InitialPosition(initialStandingAngle));
            int forward = 1;
            int right = 2;
            int backward = 3;
            int left = 4;
            dict.Add("forward", (x) => Gait1Walk(forward));
            dict.Add("right", (x) => Gait1Walk(right));
            dict.Add("backward", (x) => Gait1Walk(backward));
            dict.Add("left", (x) => Gait1Walk(left));
            dict.Add("forward2", (x) => Gait2Walk(forward));
            dict.Add("right2", (x) => Gait2Walk(right));
            dict.Add("backward2", (x) => Gait2Walk(backward));
            dict.Add("left2", (x) => Gait2Walk(left));
            dict.Add("forwardKMPC", (x) => MoveRobotByKMPC(forward, x[0]));
            dict.Add("rightKMPC", (x) => MoveRobotByKMPC(right, x[0]));
            dict.Add("backwardKMPC", (x) => MoveRobotByKMPC(backward, x[0]));
            dict.Add("leftKMPC", (x) => MoveRobotByKMPC(left, x[0]));
            dict.Add("initialAccurate", (x) => AccurateInitialPosition(initialStandingAngle));
            dict.Add("sitDown", (x) => SitDown(x[0]));
            dict.Add("initManipulation", (x) => TeleoperationPosition(initialStandingAngle));
            dict.Add("manipulation", (x) => ManipulateDirect(x[0], x[1], x[2]));
            dict.Add("manipulationPP", (x) => ManipulatePP(x[0], x[1], x[2]));
            manipulator.LoadCSV();
            Console.WriteLine("Node loaded");
        }
        
        public void ExecuteCMD(string cmd)
        {
            var parts = cmd.Split('_');
            string key = parts[0];
            int[] value = new int[3]; // default 0,0,0

            try
            {
                if (parts.Length == 4)
                {
                    value[0] = int.Parse(parts[1]);
                    value[1] = int.Parse(parts[2]);
                    value[2] = int.Parse(parts[3]);
                }
                else if (parts.Length == 2)
                {
                    value[0] = int.Parse(parts[1]);
                }
                // else: value remains [0,0,0]

                Console.WriteLine($"Executing: {key} with values [{value[0]}, {value[1]}, {value[2]}]");

                if (_controlRights && dict.ContainsKey(key))
                {
                    dict[key](value);
                }
                else if (!_controlRights)
                {
                    Console.WriteLine("Wait for previous execution");
                }
                else
                {
                    Console.WriteLine($"Unknown command: {key}");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Command parse error: {ex.Message}");
            }
        }
/*
                        public void ExecuteCMD(string cmd)
                        {
                            var command = cmd.Split("_");
                            int[] value =new int[3];
                            value[0] = 30;
                            if (command.Length == 4)
                            {
                                value[0] = int.Parse(command[1]);
                                value[1] = int.Parse(command[2]);
                                value[2] = int.Parse(command[3]);
                                Console.WriteLine(command[0] + " " + command[1] + " " + command[2] + " " + command[3]);
                                Console.WriteLine(command[0] + " " + value[0] + " " + value[1] + " " + value[2]);
                            }
                            else if (command.Length == 2)
                            {
                                value[0] = int.Parse(command[1]);
                                Console.WriteLine(command[0] + " " + command[1]);
                            }
                            else
                            {
                                command[0] = cmd;
                            }
                            if (_controlRights)
                                dict[command[0]](value);
                            else
                                Console.WriteLine("wait for previous execution");
                        }*/

        public async void MoveRobotBySequence(List<sbyte> angles)
        {
            await _awaitRights(1);
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Walking KMPC Trajectories");
            await Task.Delay(100);
            if (CommunicatorDotNet.GetAngleDelegate(0) != 0 || CommunicatorDotNet.GetAngleDelegate(1) != 0 || CommunicatorDotNet.GetAngleDelegate(2) != 0 || CommunicatorDotNet.GetAngleDelegate(3) != 0)
                await _InitialPosition(token);
            var task = _ExecuteTrajectory(angles, token);
            var delayTask = Task.Delay(60000); //timeout
            await Task.WhenAny(task, delayTask);
            cts.Cancel();
            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES
            await Task.Delay(100);
            //await Task.Run(_InitialPosition);
            CommunicatorDotNet.Request_KMPC_Delegate(new Empty());
            Console.WriteLine("KMPC Trajectory executed");
            _controlRights = true;
        }

        public async void MoveRobotByKMPC(int direction, int angle)
        {
            await _awaitRights();
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            //await Task.Delay(100);
            if (((CommunicatorDotNet.GetAngleDelegate(direction - 1) != 0 || CommunicatorDotNet.GetAngleDelegate((-1 + direction + 2) % 4) != 0)))
            {//Console.WriteLine("1");
                await _InitialPosition(token);
            }
            else if ((CommunicatorDotNet.GetAngleDelegate(direction + 4 - 1) != -initialStandingAngle || CommunicatorDotNet.GetAngleDelegate((-1 + direction + 2) % 4 + 4) != -initialStandingAngle))
            {//Console.WriteLine("2");
                await _InitialPosition(token);
            }
            else if ((CommunicatorDotNet.GetAngleDelegate(-1 + direction + 8) != -90 + initialStandingAngle || CommunicatorDotNet.GetAngleDelegate((-1 + direction + 2) % 4 + 8) != -90 + initialStandingAngle))
            {//Console.WriteLine("3");
                await _InitialPosition(token);
            }
            await Task.Delay(500);
            Console.WriteLine("Walking KMPC Gait 1 Trajectory");
            var task = _Gait1Walk(direction, token, angle);
            var delayTask = Task.Delay(30000); //timeout
            await Task.WhenAny(task, delayTask);
            cts.Cancel();
            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            await Task.Delay(100);
            CommunicatorDotNet.Request_KMPC_Delegate(new Empty());
            Console.WriteLine("KMPC Trajectory executed");
            _controlRights = true;
        }

        private async void InitialPosition(int initialAngle)
        {
            await _awaitRights();
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Initial Position");
            var task = _InitialPosition(token);
            var delayTask = Task.Delay(20000); //timeout
            await Task.WhenAny(task, delayTask);
            cts.Cancel();

            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("Initial Position executed");
            _controlRights = true;
        }

        private async void AccurateInitialPosition(int initialAngle)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Accurate Initial Position");
            var task = _AccurateInitialPosition(token);
            var delayTask = Task.Delay(20000); //timeout
            await Task.WhenAny(task, delayTask);
            cts.Cancel();

            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("Accurate Initial Position executed");
            _controlRights = true;
        }

        private bool _tPosition = false;
        
        private async void TeleoperationPosition(int initialAngle)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Teleoperation Position executing");
            if (!_tPosition)
            {


                var task = _InitialPosition(token);
                var delayTask = Task.Delay(20000); //timeout
                await Task.WhenAny(task, delayTask);
                var task1 = _TeleoperationPositionOn(token);
                var delayTask1 = Task.Delay(20000); //timeout
                await Task.WhenAny(task1, delayTask1);
                _tPosition = true;
            }
            else
            {


                var task1 = _TeleoperationPositionOff(token);
                var delayTask1 = Task.Delay(20000); //timeout
                await Task.WhenAny(task1, delayTask1);
                _tPosition = false;
            }
            cts.Cancel();
            

            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("Teleoperation Position executed");
            _controlRights = true;
        }


        private async void ManipulateDirect(int UnityX, int UnityY, int UnityZ)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Manipulation executing");
            if (manipulator == null)
            {
                Console.WriteLine("Can't manipulate, no manipulator");
                cts = null;
                token = CancellationToken.None; //CLEAR VARIABLES
                _controlRights = true;
                return;
            }
            if (_tPosition)
            {
                Vector3 target = new Vector3((float)UnityX/1000, (float)UnityY/1000, (float)UnityZ/1000);
                //Console.WriteLine(target.X+" "+target.Y+" "+target.Z);
                var task1 = _ManipulateDirect(target, token);
                var delayTask1 = Task.Delay(20000); //timeout
                await Task.WhenAny(task1, delayTask1);
            }
            else
            {
                Console.WriteLine("Can't manipulate, not in position");
            }
            cts.Cancel();


            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("Manipulation executed");
            _controlRights = true;
        }

        private async Task _ManipulateDirect(Vector3 target, CancellationToken token)
        {   //Console.WriteLine("_ManipulateDirect: "+target.X +" " + target.Y+ " " + target.Z);
            var Joints = manipulator.DirectManipulation(target);
            Console.WriteLine(Joints[0]+" "+Joints[1]+" "+Joints[2]);
            var task1 = motors[0, 0].RotateTarget(Joints[0], token);
            var task2 = motors[0, 1].RotateTarget(Joints[1], token);
            var task3 = motors[0, 2].RotateTarget(Joints[2], token);
            await Task.WhenAll(task1, task2, task3);
            await Task.Yield();
        }



        private async void ManipulatePP(int UnityX, int UnityY, int UnityZ)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("ManipulationPP executing");
            if (manipulator == null)
            {
                Console.WriteLine("Can't manipulate, no manipulator");
                cts = null;
                token = CancellationToken.None; //CLEAR VARIABLES
                _controlRights = true;
                return;
            }
            if (_tPosition)
            {
                Vector3 target = new Vector3((float)UnityX/1000, (float)UnityY/1000, (float)UnityZ/1000);
                //Console.WriteLine(target.X+" "+target.Y+" "+target.Z);
                var task1 = _ManipulatePP(target, token);
                var delayTask1 = Task.Delay(20000); //timeout
                await Task.WhenAny(task1, delayTask1);
            }
            else
            {
                Console.WriteLine("Can't manipulate, not in position");
            }
            cts.Cancel();


            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("ManipulationPP executed");
            _controlRights = true;
        }

        private async Task _ManipulatePP(Vector3 target, CancellationToken token)
        {   //Console.WriteLine("_ManipulateDirect: "+target.X +" " + target.Y+ " " + target.Z);
            
            var curState = motors[0,0].CurrentPrimaryAxisRotation();
            var curState1 = motors[0,1].CurrentPrimaryAxisRotation();
            var curState2= motors[0,2].CurrentPrimaryAxisRotation();
            var FK = ForwardKinematicsFull(curState, curState1, curState2, L_Sh, L_UA, L_FA);
           
            Vector3 current = new(-(float)FK.position[1]/1000,(float)FK.position[2]/1000, (float)FK.position[0]/1000);
            Console.WriteLine("Current Pose: " + current.X + " "+ current.Y + " "+ current.Z);
            var Joints = manipulator.ManipulationPP(target,current);
            foreach (var joints in Joints)
            {
                var task1 = motors[0, 0].RotateTarget(joints.X, token);
                var task2 = motors[0, 1].RotateTarget(joints.Y, token);
                var task3 = motors[0, 2].RotateTarget(joints.Z, token);
                await Task.WhenAll(task1, task2, task3);
            }
           
            await Task.Yield();
        }



        private async void Gait1Walk(int direction)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Walking with Gait 1");
            if (CommunicatorDotNet.GetAngleDelegate((direction + 1) % 4) != 0 || CommunicatorDotNet.GetAngleDelegate((direction + 3) % 4) != 0)
                await _InitialPosition(token);
            var task = _Gait1Walk(direction, token);
            var delayTask = Task.Delay(30000);
            await Task.WhenAny(task, delayTask); //timeout
            cts.Cancel();

            cts = null;
            token = CancellationToken.None;

            //await Task.Run(()=>_Gait1Walk(direction));
            Console.WriteLine("Walking with Gait 1 executed");
            _controlRights = true;
        }

        private async void Gait2Walk(int direction)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Walking with Gait 2");

            var task = _Gait2Walk(direction, token);
            var delayTask = Task.Delay(20000);
            await Task.WhenAny(task, delayTask); //timeout
            cts.Cancel();

            cts = null;
            token = CancellationToken.None;

            //await Task.Run(()=>_Gait1Walk(direction));
            Console.WriteLine("Walking with Gait 2 executed");
            _controlRights = true;
        }

        private async void SitDown(int num)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Prepare for take off");

            var task = _SitDown(token);
            var delayTask = Task.Delay(20000);
            await Task.WhenAny(task, delayTask); //timeout
            cts.Cancel();

            cts = null;
            token = CancellationToken.None;

            //await Task.Run(()=>_Gait1Walk(direction));
            Console.WriteLine("I am sitting, ready to take off");
            _controlRights = true;
        }



        private async Task _awaitRights(int time = 3)
        {
            int i = 0;

            while (!_controlRights)
            {
                await Task.Delay(10);
                i++;
                if (i > time * 100)
                    return;
            }
        }
        private async Task _InitialPosition(CancellationToken token)
        {
            Console.WriteLine("Initial Position");
            var angle1 = -initialStandingAngle;
            var angle2 = -90 + initialStandingAngle;
            await Task.Delay(10);

            var task1 = motors[0, 1].RotateTarget(angle1, token);
            var task2 = motors[0, 2].RotateTarget(angle2, token);

            if (Math.Abs(motors[0, 0].CurrentPrimaryAxisRotation()) > 3)
            {
                // motors[0,1].RotateTarget(0,token);
                // await motors[0,2].RotateTarget(-90,token);
                task1 = motors[0, 1].RotateTarget(0, token);
                task2 = motors[0, 2].RotateTarget(-90, token);
                await Task.WhenAll(task1, task2);

                await motors[0, 0].RotateTarget(0, token);

                task1 = motors[0, 1].RotateTarget(angle1, token);
                task2 = motors[0, 2].RotateTarget(angle2, token);
                await Task.WhenAll(task1, task2);
                // motors[0,1].RotateTarget(angle1,token);
                // await motors[0,2].RotateTarget(angle2,token);
            }
            else
            {
                motors[0, 0].RotateTarget(0, token);
                task1 = motors[0, 1].RotateTarget(angle1, token);
                task2 = motors[0, 2].RotateTarget(angle2, token);
                await Task.WhenAll(task1, task2);
                // motors[0,1].RotateTarget(angle1,token);
                // await motors[0,2].RotateTarget(angle2,token);
            }


            if (Math.Abs(motors[2, 0].CurrentPrimaryAxisRotation()) > 3)
            {
                task1 = motors[2, 1].RotateTarget(0, token);
                task2 = motors[2, 2].RotateTarget(-90, token);
                await Task.WhenAll(task1, task2);
                // motors[2,1].RotateTarget(0,token);
                // await motors[2,2].RotateTarget(-90,token);
                await motors[2, 0].RotateTarget(0, token);
                task1 = motors[2, 1].RotateTarget(angle1, token);
                task2 = motors[2, 2].RotateTarget(angle2, token);
                await Task.WhenAll(task1, task2);
                // motors[2,1].RotateTarget(angle1,token);
                // await motors[2,2].RotateTarget(angle2,token);
            }
            else
            {
                motors[2, 0].RotateTarget(0, token);
                task1 = motors[2, 1].RotateTarget(angle1, token);
                task2 = motors[2, 2].RotateTarget(angle2, token);
                await Task.WhenAll(task1, task2);
                // motors[2,1].RotateTarget(angle1,token);
                // await motors[2,2].RotateTarget(angle2,token);
            }


            if (Math.Abs(motors[3, 0].CurrentPrimaryAxisRotation()) > 3)
            {
                task1 = motors[3, 1].RotateTarget(0, token);
                task2 = motors[3, 2].RotateTarget(-90, token);
                await Task.WhenAll(task1, task2);
                // motors[3,1].RotateTarget(0,token);
                // await motors[3,2].RotateTarget(-90,token);
                await motors[3, 0].RotateTarget(0, token);
                task1 = motors[3, 1].RotateTarget(angle1, token);
                task2 = motors[3, 2].RotateTarget(angle2, token);
                await Task.WhenAll(task1, task2);
                // motors[3,1].RotateTarget(angle1,token);
                // await motors[3,2].RotateTarget(angle2,token);
            }

            else
            {
                motors[3, 0].RotateTarget(0, token);
                task1 = motors[3, 1].RotateTarget(angle1, token);//;
                task2 = motors[3, 2].RotateTarget(angle2, token);//;
                await Task.WhenAll(task1, task2);
                // motors[3,1].RotateTarget(angle1,token);//;
                // await motors[3,2].RotateTarget(angle2,token);//;
            }

            if (Math.Abs(motors[1, 0].CurrentPrimaryAxisRotation()) > 3)
            {
                task1 = motors[1, 1].RotateTarget(0, token);
                task2 = motors[1, 2].RotateTarget(-90, token);
                await Task.WhenAll(task1, task2);
                // motors[1,1].RotateTarget(0,token);
                // await motors[1,2].RotateTarget(-90,token);
                await motors[1, 0].RotateTarget(0, token);
                task1 = motors[1, 1].RotateTarget(angle1, token);
                task2 = motors[1, 2].RotateTarget(angle2, token);//;
                await Task.WhenAll(task1, task2);
                // motors[1,1].RotateTarget(angle1,token);
                // await motors[1,2].RotateTarget(angle2,token);//;
            }
            else
            {
                motors[1, 0].RotateTarget(0, token);
                task1 = motors[1, 1].RotateTarget(angle1, token);
                task2 = motors[1, 2].RotateTarget(angle2, token);//;
                await Task.WhenAll(task1, task2);
                // motors[1,1].RotateTarget(angle1,token);
                // await motors[1,2].RotateTarget(angle2,token);//;
            }
            await Task.Yield();
        }


        private async Task _AccurateInitialPosition(CancellationToken token)
        {
            Console.WriteLine("Accurate Initial Position Execution");
            var angle1 = -initialStandingAngle;
            var angle2 = -90 + initialStandingAngle;
            await Task.Delay(10);
            int direction = 2;
            int forward = direction - 1;
            int right = (direction) % 4;
            int back = (direction + 1) % 4;
            int left = (direction + 2) % 4;


            //Forward
            var task1 = motors[forward, 1].RotateTarget(-15, token);
            var task2 = motors[forward, 2].RotateTarget(-75, token);
            await Task.WhenAll(task1, task2);

            await motors[forward, 0].RotateTarget(0, token);

            task1 = motors[forward, 1].RotateTarget(angle1, token);
            task2 = motors[forward, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);
            /*----------------------------------------*/
            task1 = motors[back, 1].RotateTarget(-15, token);
            task2 = motors[back, 2].RotateTarget(-75, token);
            await Task.WhenAll(task1, task2);

            await motors[back, 0].RotateTarget(0, token);

            task1 = motors[back, 1].RotateTarget(angle1, token);
            task2 = motors[back, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);
            /*----------------------------------------*/
            task1 = motors[left, 1].RotateTarget(-15, token);
            task2 = motors[left, 2].RotateTarget(-75, token);
            await Task.WhenAll(task1, task2);

            await motors[left, 0].RotateTarget(0, token);

            task1 = motors[left, 1].RotateTarget(angle1, token);
            task2 = motors[left, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);
            /*----------------------------------------*/

            task1 = motors[right, 1].RotateTarget(-15, token);
            task2 = motors[right, 2].RotateTarget(-75, token);
            await Task.WhenAll(task1, task2);

            await motors[right, 0].RotateTarget(0, token);

            task1 = motors[right, 1].RotateTarget(angle1, token);
            task2 = motors[right, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);

            await Task.Yield();
        }

        private async Task _TeleoperationPositionOn(CancellationToken token)
        {
            Console.WriteLine("Initial Position");
            var angle1 = -initialStandingAngle;
            var angle2 = -90 + initialStandingAngle;
            await Task.Delay(10);

            var task1 = motors[0, 1].RotateTarget(angle1, token);
            var task2 = motors[0, 2].RotateTarget(angle2, token);

            //Right
            task1 = motors[1, 1].RotateTarget(0, token);
            task2 = motors[1, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            await motors[1, 0].RotateTarget(30, token);

            task1 = motors[1, 1].RotateTarget(angle1, token);
            task2 = motors[1, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);


            //Left
            task1 = motors[3, 1].RotateTarget(0, token);
            task2 = motors[3, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            await motors[3, 0].RotateTarget(-30, token);

            task1 = motors[3, 1].RotateTarget(angle1, token);
            task2 = motors[3, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);



            //Sit for better workspace
            task1 = motors[2, 1].RotateTarget(0, token);
            task2 = motors[2, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            //Give hand
            task1 = motors[0, 1].RotateTarget(0, token);
            task2 = motors[0, 2].RotateTarget(0, token);
            await Task.WhenAll(task1, task2);

            await Task.Yield();
        }



        private async Task _TeleoperationPositionOff(CancellationToken token)
        {
            Console.WriteLine("Initial Position");
            var angle1 = -initialStandingAngle;
            var angle2 = -90 + initialStandingAngle;
            await Task.Delay(10);

            var task1 = motors[0, 1].RotateTarget(angle1, token);
            var task2 = motors[0, 2].RotateTarget(angle2, token);

            task1 = motors[0, 1].RotateTarget(0, token);
            task2 = motors[0, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            await motors[0, 0].RotateTarget(0, token);

            task1 = motors[0, 1].RotateTarget(angle1, token);
            task2 = motors[0, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);



            await motors[2, 0].RotateTarget(0, token);
            task1 = motors[2, 1].RotateTarget(angle1, token);
            task2 = motors[2, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);



            task1 = motors[3, 1].RotateTarget(0, token);
            task2 = motors[3, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            await motors[3, 0].RotateTarget(0, token);
            task1 = motors[3, 1].RotateTarget(angle1, token);
            task2 = motors[3, 2].RotateTarget(angle2, token);
            await Task.WhenAll(task1, task2);



            task1 = motors[1, 1].RotateTarget(0, token);
            task2 = motors[1, 2].RotateTarget(-90, token);
            await Task.WhenAll(task1, task2);

            await motors[1, 0].RotateTarget(0, token);
            task1 = motors[1, 1].RotateTarget(angle1, token);
            task2 = motors[1, 2].RotateTarget(angle2, token);//;
            await Task.WhenAll(task1, task2);

            await Task.Yield();
        }






        private async Task _Gait1Walk(int direction, CancellationToken token, int angle = 30)
        {
            token.ThrowIfCancellationRequested();

            int forward = direction - 1;
            int right = (direction) % 4;
            int back = (direction + 1) % 4;
            int left = (direction + 2) % 4;
            /*
            if (Math.Abs(motors[back,0].CurrentPrimaryAxisRotation()) > 1)
            {
                await motors[back,1].RotateTarget(0,token);
                await motors[back,0].RotateTarget(0,token);
                await motors[back,1].RotateTarget(-initialStandingAngle,token);
                motors[back,2].RotateTarget(-90 + initialStandingAngle,token);
            }
            if (Math.Abs(motors[forward,0].CurrentPrimaryAxisRotation()) > 1)
            {
                await motors[forward,1].RotateTarget(0,token);
                await motors[forward,0].RotateTarget(0,token);
                await motors[forward,1].RotateTarget(-initialStandingAngle,token);
                motors[forward,2].RotateTarget(-90 + initialStandingAngle,token);
            }*/

            Halfstep(right, true, token, angle);
            await Halfstep(left, false, token, angle);
            await Task.Delay(10);
            await Stepper(right, left, back, forward, token, angle);


            motors[back, 0].RotateTarget(0, token);
            motors[back, 1].RotateTarget(-initialStandingAngle, token);
            await motors[back, 2].RotateTarget(-90 + initialStandingAngle, token);

            await Task.Delay(10);

            motors[forward, 0].RotateTarget(0, token);
            motors[forward, 1].RotateTarget(-initialStandingAngle, token);
            await motors[forward, 2].RotateTarget(-90 + initialStandingAngle, token);

            await Task.Delay(10);
            await Task.Yield();
        }

        private async Task _Gait2Walk(int direction, CancellationToken token)
        {
            token.ThrowIfCancellationRequested();

            await _InitialPosition(token); //Initial walk position

            await Walk(direction, token);
        }

        private async Task _ExecuteTrajectory(List<sbyte> angles, CancellationToken token)
        {

            for (int i = 0; i < 200; i++)
            {

                if (token.IsCancellationRequested)
                    return;
                motors[0, 0].RotateTo(0);
                motors[1, 0].RotateTo(0);
                motors[2, 0].RotateTo(0);
                motors[3, 0].RotateTo(0);


                motors[0, 1].RotateTo(angles[i]);
                motors[0, 2].RotateTo(angles[200 + i]);
                motors[1, 1].RotateTo(angles[400 + i]);
                motors[1, 2].RotateTo(angles[600 + i]);
                motors[2, 1].RotateTo(angles[800 + i]);
                motors[2, 2].RotateTo(angles[1000 + i]);
                motors[3, 1].RotateTo(angles[1200 + i]);
                motors[3, 2].RotateTo(angles[1400 + i]);

                await Task.Delay(10);
                await Task.Yield();
            }
        }

        private async Task Walk(int direction, CancellationToken token)
        {
            int _leftForward = direction - 1;  //forward
            int _rightForward = (direction) % 4; //right
            int _rightHind = (direction + 1) % 4; //back
            int _leftHind = (direction + 2) % 4; //left


            int trajectoryHalfPointNumber = 200;
            float[][] trajectory = trajectoryGenerator(trajectoryHalfPointNumber, 0.9f, 0.5f);
            float[][] angles = inverseKinematics(trajectory[0], trajectory[1]);
            int i = (int)trajectoryHalfPointNumber / 2;
            //float j = 0;
            int j = 0;
            bool loop = true;

            while (loop)
            {
                if (token.IsCancellationRequested)
                    return;

                motors[_rightForward, 1].RotateTo((sbyte)angles[0][i]);//, token); // Start loop
                motors[_rightForward, 2].RotateTo((sbyte)angles[1][i]);//, token);
                motors[_leftForward, 1].RotateTo((sbyte)angles[0][(i + trajectoryHalfPointNumber) % (2 * trajectoryHalfPointNumber)]);//, token); //Start from half loop
                motors[_leftForward, 2].RotateTo((sbyte)angles[1][(i + trajectoryHalfPointNumber) % (2 * trajectoryHalfPointNumber)]);//, token);
                motors[_rightHind, 1].RotateTo((sbyte)angles[0][2 * trajectoryHalfPointNumber - 1 - i]);//, token); //Start backloop
                motors[_rightHind, 2].RotateTo((sbyte)angles[1][2 * trajectoryHalfPointNumber - 1 - i]);//, token);
                motors[_leftHind, 1].RotateTo((sbyte)angles[0][(3 * trajectoryHalfPointNumber - i) % (2 * trajectoryHalfPointNumber)]);//, token);
                motors[_leftHind, 2].RotateTo((sbyte)angles[1][(3 * trajectoryHalfPointNumber - i) % (2 * trajectoryHalfPointNumber)]);//, token);


                i++;

                if ((int)(i / (2 * trajectoryHalfPointNumber)) == 1)
                {
                    j++;
                    if (j == 3) // Step Count
                    {
                        await _InitialPosition(token);
                        break;
                    }

                }


                i = i % (2 * trajectoryHalfPointNumber);

                await Task.Delay(10);
                await Task.Yield();
            }
        }

        public float[][] trajectoryGenerator(int segment_num, float multiplicator = 0.8f, float _archimed_compression = 0.7f)
        {
            var _robotStandingHeight = (float)Math.Sin(initialStandingAngle * Math.PI / 180) * L_UA + L_FA - 10;
            var _step_shift_forward = (int)(Math.Sqrt((L_UA + L_FA) * (L_UA + L_FA) - _robotStandingHeight * _robotStandingHeight) * 0.9f);         //Leg is straight
            //float _archimed_shift = 0.7f;
            //float _archimed_compression = 0.7f;
            List<float> x = new List<float>();
            List<float> y = new List<float>();
            for (int i = 0; i < segment_num; i++) //Straight Line shifted to close archimed spiral
            {

                x.Add((float)(_step_shift_forward - multiplicator * _step_shift_forward * i / segment_num));// _archimed_shift ;
                y.Add((float)-_robotStandingHeight);
            }
            for (int j = segment_num; j > 0; j--)  //Archimed Spiral Compressed by Y
            {
                x.Add((float)-j / segment_num * multiplicator * _step_shift_forward * (float)Math.Cos(Math.PI - Math.PI * j / segment_num) + _step_shift_forward);//_archimed_shift *);
                y.Add((float)j / segment_num * _archimed_compression * _step_shift_forward * (float)Math.Sin(Math.PI - Math.PI * j / segment_num) - _robotStandingHeight);

            }
            float[] x_float = x.ToArray();
            float[] y_float = y.ToArray();

            return new[] { x_float, y_float };
        }

        public float[][] inverseKinematics(float[] x, float[] y)
        {
            float[][] theta = new float[2][];
            theta[0] = new float[x.Length];
            theta[1] = new float[y.Length];


            int L1 = L_UA;
            int L2 = L_FA;
            for (int i = 0; i < x.Length; i++)
            {
                float c2 = (x[i] * x[i] + y[i] * y[i] - L1 * L1 - L2 * L2) / (2 * L1 * L2);
                float s2 = (float)(Math.Sqrt(1 - c2 * c2));
                theta[1][i] = (float)(-Math.Acos(c2) * 180 / Math.PI);
                theta[0][i] = (float)((Math.Atan2(y[i], x[i]) + Math.Atan2(L2 * s2, L1 + L2 * c2)) * 180 / Math.PI);
            }

            return (theta);
        }



        // Helper method to convert degrees to radians
        private static double ToRadians(double degrees)
        {
            return degrees * Math.PI / 180.0;
        }

        // DH Matrix calculation
        public static double[,] DHMatrix(double theta, double d, double a, double alpha)
        {
            theta = ToRadians(theta);
            alpha = ToRadians(alpha);
            
            double cosTheta = Math.Cos(theta);
            double sinTheta = Math.Sin(theta);
            double cosAlpha = Math.Cos(alpha);
            double sinAlpha = Math.Sin(alpha);
            
            return new double[,]
            {
                {cosTheta, -sinTheta * cosAlpha,  sinTheta * sinAlpha, a * cosTheta},
                {sinTheta,  cosTheta * cosAlpha, -cosTheta * sinAlpha, a * sinTheta},
                {0,         sinAlpha,             cosAlpha,            d},
                {0,         0,                   0,                   1}
            };
        }

        // Matrix multiplication helper
        private static double[,] MatrixMultiply(double[,] a, double[,] b)
        {
            int rowsA = a.GetLength(0);
            int colsA = a.GetLength(1);
            int colsB = b.GetLength(1);
            
            double[,] result = new double[rowsA, colsB];
            
            for (int i = 0; i < rowsA; i++)
            {
                for (int j = 0; j < colsB; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < colsA; k++)
                    {
                        sum += a[i, k] * b[k, j];
                    }
                    result[i, j] = sum;
                }
            }
            
            return result;
        }

        // Extracts the position vector from transformation matrix
        private static double[] GetPosition(double[,] matrix)
        {
            return new double[] { matrix[0, 3], matrix[1, 3], matrix[2, 3] };
        }

        // Extracts the orientation matrix from transformation matrix
        private static double[,] GetOrientation(double[,] matrix)
        {
            return new double[,]
            {
                { matrix[0, 0], matrix[0, 1], matrix[0, 2] },
                { matrix[1, 0], matrix[1, 1], matrix[1, 2] },
                { matrix[2, 0], matrix[2, 1], matrix[2, 2] }
            };
        }

        // Forward kinematics calculation
        public static (double[] position, double[,] orientation, double[,] transform) 
        ForwardKinematicsFull(double theta1, double theta2, double theta3, double L1, double L2, double L3)
        {
            double[,] T1 = DHMatrix(theta1, 0, 0, 90);
            double[,] T2 = DHMatrix(theta2, 0, L1, 0);
            double[,] T3 = DHMatrix(theta3, 0, L2, 0);
            
            double[,] TEnd = new double[,]
            {
                {1, 0, 0, L3},
                {0, 1, 0, 0},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            };
            
            double[,] T = MatrixMultiply(MatrixMultiply(MatrixMultiply(T1, T2), T3), TEnd);
            
            double[] position = GetPosition(T);
            double[,] orientation = GetOrientation(T);
            
            return (position, orientation, T);
        }
    

        private async Task Halfstep(int leg, bool side, CancellationToken token, int angle)
        {
            // motors[leg,1].RotateTarget(0,token);
            // await  motors[leg,2].RotateTarget(-90,token);

            var moveSh = motors[leg, 1].RotateTarget(0, token);
            var moveUA = motors[leg, 2].RotateTarget(-90, token);
            await Task.WhenAll(moveSh, moveUA);

            if (side)
                await motors[leg, 0].RotateTarget(angle, token);
            else
                await motors[leg, 0].RotateTarget(-angle, token);
            var moveSh1 = motors[leg, 1].RotateTarget(-initialStandingAngle, token);
            var moveUA1 = motors[leg, 2].RotateTarget(-90 + initialStandingAngle, token);
            await Task.WhenAll(moveSh1, moveUA1);
            // motors[leg,1].RotateTarget(-initialStandingAngle,token);
            // await  motors[leg,2].RotateTarget(-90 + initialStandingAngle,token);
        }
        internal int L_UA = 155;
        internal int L_FA = 275;
        internal int L_Sh = 95;

        private async Task Stepper(int right, int left, int back, int forward, CancellationToken token, int angle)
        {
            var speedRotation = 1;
            var upAngle = 10;

            float targetAngle = angle;
            float initialRotationright = motors[right, 0].CurrentPrimaryAxisRotation();
            float initialRotationleft = motors[left, 0].CurrentPrimaryAxisRotation();
            var loop = true;
            float deltaAngle = targetAngle - initialRotationleft;
            float rotationTotal;
            float rotation;
            float l_max = (float)(L_UA * Math.Cos(initialStandingAngle * Math.PI / 180) + L_Sh);  //Projection on the floor L_shoulder+L_FA*cos(theta1^init)

            motors[left, 0].rotationState = 1;
            motors[right, 0].rotationState = -1;

            while (loop)
            {
                if (token.IsCancellationRequested)
                    return;


                float rotationChange = (float)motors[left, 0].rotationState * speedRotation; //*dt
                float rotationGoalleft = motors[left, 0].CurrentPrimaryAxisRotation() + rotationChange;
                float rotationGoalright = motors[right, 0].CurrentPrimaryAxisRotation() - rotationChange;
                rotationTotal = Math.Abs(initialRotationleft - rotationGoalleft);


                if (rotationTotal < targetAngle)
                    rotation = rotationTotal;
                else
                    rotation = 2 * targetAngle - rotationTotal;

                //FROM ARTICLES with mistake* - length of the shoulder joint wasn't considered
                //float rotationGoalHeight = Mathf.Acos((2 * Mathf.Cos(rotation * Mathf.Deg2Rad) - 1) * Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad)) * Mathf.Rad2Deg - initialStandingAngle;

                float rotationGoalHeight = (float)(Math.Acos(2 * (Math.Cos(rotation * Math.PI / 180) - 1) * l_max / L_UA + Math.Cos(initialStandingAngle * Math.PI / 180)) * 180 / Math.PI - initialStandingAngle);


                if ((rotationGoalleft < motors[left, 0].lowerLimit) | (rotationGoalleft > motors[left, 0].upperLimit)) //Check limitations
                    return;


                if (rotationTotal > Math.Abs(deltaAngle))  //Fixing angles
                {
                    loop = false;
                    motors[right, 0].RotateTo((sbyte)(initialRotationright - deltaAngle));
                    motors[left, 0].RotateTo((sbyte)(initialRotationleft + deltaAngle));
                    motors[right, 1].RotateTo((sbyte)(-initialStandingAngle));
                    motors[right, 2].RotateTo((sbyte)(-90 + initialStandingAngle));
                    motors[left, 1].RotateTo((sbyte)(-initialStandingAngle));
                    motors[left, 2].RotateTo((sbyte)(-90 + initialStandingAngle));


                    await motors[back, 2].RotateTarget((sbyte)(-90 + initialStandingAngle), token);
                    motors[back, 1].RotateTarget((sbyte)(-initialStandingAngle), token);

                    //forward[1].RotateTo(-initialStandingAngle + upAngle);


                    motors[forward, 2].RotateTarget((sbyte)(-90 + initialStandingAngle), token);
                    await motors[forward, 1].RotateTarget((sbyte)(-initialStandingAngle), token);

                    return;
                }
                //else


                motors[right, 0].RotateTo((sbyte)(rotationGoalright));
                motors[left, 0].RotateTo((sbyte)(rotationGoalleft));
                motors[right, 1].RotateTo((sbyte)(-initialStandingAngle - rotationGoalHeight));
                motors[right, 2].RotateTo((sbyte)(-90 + initialStandingAngle + rotationGoalHeight));
                motors[left, 1].RotateTo((sbyte)(-initialStandingAngle - rotationGoalHeight));
                motors[left, 2].RotateTo((sbyte)(-90 + initialStandingAngle + rotationGoalHeight));

                if (rotationGoalleft < 0)
                    motors[back, 2].RotateTarget((sbyte)(-90 + initialStandingAngle + upAngle * rotationTotal / deltaAngle), token);
                else
                    motors[back, 1].RotateTo((sbyte)(-initialStandingAngle + 0.5f * rotationGoalleft / targetAngle * upAngle));   //upAngle * rotationTotal / deltaAngle);



                motors[forward, 1].RotateTo((sbyte)(-initialStandingAngle + upAngle * rotationTotal / deltaAngle));// +rotationGoalHeight);
                motors[forward, 2].RotateTo((sbyte)(-90 + initialStandingAngle - 0.5f * upAngle * rotationTotal / deltaAngle));


                await Task.Delay(15);
                await Task.Yield();
            }

        }


        private async Task _SitDown(CancellationToken token)
        {
            int num = 10;
            var landAngles = LandTrajectory(num);

            int i = 0;
            foreach (var angles in landAngles)
            {
                //Console.WriteLine($"Th1: {angles[0]}, Th2: {angles[1]}");
                i++;
                if (i < num / 3)
                    continue;

                if (token.IsCancellationRequested)
                    return;

                var task1 = motors[0, 1].RotateTarget(angles[0], token);
                var task2 = motors[0, 2].RotateTarget(angles[1], token);
                var task3 = motors[1, 1].RotateTarget(angles[0], token);
                var task4 = motors[1, 2].RotateTarget(angles[1], token);
                var task5 = motors[2, 1].RotateTarget(angles[0], token);
                var task6 = motors[2, 2].RotateTarget(angles[1], token);
                var task7 = motors[3, 1].RotateTarget(angles[0], token);
                var task8 = motors[3, 2].RotateTarget(angles[1], token);
                await Task.WhenAll(task1, task2, task3, task4, task5, task6, task7, task8);
                //await Task.Delay(100);
                await Task.Yield();
            }
        }


        private List<List<sbyte>> LandTrajectory(int segmentNum)
        {
            var landAngles = new List<List<sbyte>>();
            double desire = Math.Sqrt(Math.Pow(L_UA + L_FA, 2) - Math.Pow(L_UA, 2));

            for (int i = 0; i < segmentNum; i++)
            {
                double y = -desire + (desire - L_FA) * (i + 1) / segmentNum;
                landAngles.Add(InverseKinematics(L_UA, y));
            }

            return landAngles;
        }

        private List<sbyte> InverseKinematics(double x, double y)
        {
            double c2 = (Math.Pow(x, 2) + Math.Pow(y, 2) - Math.Pow(L_UA, 2) - Math.Pow(L_FA, 2)) / (2 * L_UA * L_FA);
            c2 = Math.Clamp(c2, -1.0, 1.0);  // Clamp value to avoid floating-point issues
            // UPD max value to avoid floating-point issues
            double s2 = Math.Sqrt(Math.Max(0.0, 1 - Math.Pow(c2, 2))); //Math.Sqrt(1 - Math.Pow(c2, 2));
            double th2 = -Math.Acos(c2) * 180 / Math.PI;
            double th1 = (Math.Atan2(y, x) + Math.Atan2(L_FA * s2, L_UA + L_FA * c2)) * 180 / Math.PI;

            return new List<sbyte> { (sbyte)th1, (sbyte)th2 };
        }



    }





    public class Mover
    {
        public Mover(int lowerLimits, int upperLimits, int idx)
        {
            lowerLimit = lowerLimits;
            upperLimit = upperLimits;
            index = idx;
        }
        private int index;
        public int lowerLimit;
        public int upperLimit;

        public int rotationState = 0;

        private bool loop = true;

        public async Task RotateTarget(float targetAngle, CancellationToken token)//, float speedRotation)
        {
            float initialRotation = CurrentPrimaryAxisRotation();
            loop = true;
            float deltaAngle = targetAngle - initialRotation;
            int speedRotation = 1;
            // if ((Math.Abs(deltaAngle)>10)&(Math.Abs(deltaAngle)<30))
            //     speedRotation = 3;
            // else if (Math.Abs(deltaAngle)>30)
            //     speedRotation = 5;

            float rotationTotal = 0.0f;


            if (deltaAngle < 0.0f)
                rotationState = -1;
            else
                rotationState = 1;

            while (loop)
            {
                if (token.IsCancellationRequested)
                    return;
                float rotationChange = rotationState * speedRotation;
                float rotationGoal = CurrentPrimaryAxisRotation() + rotationChange;
                rotationTotal = Math.Abs(initialRotation - rotationGoal);

                //Console.WriteLine( rotationChange +" Goal "+ rotationGoal +" Total "+ rotationTotal);

                if ((rotationGoal < lowerLimit) | (rotationGoal > upperLimit))
                {
                    //Console.WriteLine(rotationGoal + "  1");
                    return;
                }


                if (rotationTotal > Math.Abs(deltaAngle))
                {

                    var final = (sbyte)(initialRotation + deltaAngle);
                    await Task.Delay(5);
                    //Console.WriteLine(rotationGoal + "  2");
                    RotateTo(final);
                    return;
                }
                else
                    RotateTo((sbyte)rotationGoal);

                await Task.Delay(5);
                await Task.Yield();
            }
        }

        public float CurrentPrimaryAxisRotation()
        {
            float currentRotation = CommunicatorDotNet.GetAngleDelegate(index);
            //float currentRotation = (180 /(float) Math.PI)   * currentRotationRads;
            return currentRotation;
        }

        public void RotateTo(sbyte primaryAxisRotation)
        {
            primaryAxisRotation = (sbyte)Math.Clamp(primaryAxisRotation, lowerLimit, upperLimit);
            CommunicatorDotNet.SetAngleDelegate(primaryAxisRotation, index);
        }

    }


    public class Manipulator
    {
        internal class PositionEntry
        {
            public float time;
            public string objectName;
            public Vector3 position;
            public Vector3 joint;
        } 

        private KDTree<int> kdTree;
        internal List<PositionEntry> positionData;
        public Manipulator(int direction = 0)
        {
            kdTree = new();
            positionData = new List<PositionEntry>();
        }


        public int[] DirectManipulation(Vector3 target)
        {
            
            int nearestIndex = kdTree.FindNearest(new float[] { target.X, target.Y, target.Z });
            Console.WriteLine(nearestIndex);
            int[] jointState = new int[]{(int) positionData[nearestIndex].joint.X,
                                         (int) positionData[nearestIndex].joint.Y,
                                         (int) positionData[nearestIndex].joint.Z};

            return jointState;
        }

        public List<Vector3> ManipulationPP(Vector3 target, Vector3 current)
        {
            List<Vector3> joints = new();
            int steps = 5;
            float nudgeAmount = 0.05f;  // Small adjustment to try avoiding collision

            for (int i = 1; i <= steps; i++)
            {
                Vector3 interp = Vector3.Lerp(current, target, i / (float)steps);

                // Attempt to resolve collision by nudging around
                Vector3[] nudges = new Vector3[]
                {
                    new Vector3(0, 0, 0),                         // Original path
                    new Vector3(0, nudgeAmount, 0),        // Up
                    new Vector3(0, -nudgeAmount, 0),       // Down
                    new Vector3(nudgeAmount, 0, 0),        // Right
                    new Vector3(-nudgeAmount, 0, 0),       // Left
                    new Vector3(0, 0, nudgeAmount),        // Forward
                    new Vector3(0, 0, -nudgeAmount),       // Back
                };

                bool foundPath = false;
                foreach (var nudge in nudges)
                {
                    Vector3 tryPoint = interp + nudge;
                    if (!IsCollision(tryPoint))
                    {
                        int idx = kdTree.FindNearest(new float[] { tryPoint.X, tryPoint.Y, tryPoint.Z });
                        Vector3 joint = positionData[idx].joint;
                        joints.Add(joint);
                        // Optionally move here or store for later movement
                        // e.g., MoveTo(joint); or queue it

                        foundPath = true;
                        break;
                    }
                }

                if (!foundPath)
                {
                    Console.WriteLine($"No valid path at step {i}, even after nudging.");
                    // Optionally continue or break depending on how strict you want to be
                    continue;
                }
            }
            
            return joints;
        }


            // List<Vector3> joints = new();
            // int steps = 5;  // More = smoother
            // for (int i = 1; i <= steps; i++)
            // {
            //     Vector3 interp = Vector3.Lerp(current, target, i / (float)steps);

            //     if (IsCollision(interp))
            //     {
            //         Console.WriteLine($"Path blocked at step {i} at {interp}");
            //         break;  // Stop planning
            //     }

            //     int idx = kdTree.FindNearest(new float[] { interp.X, interp.Y, interp.Z });
            //     Vector3 joint = positionData[idx].joint;
            //     joints.Add(joint);

            // }




        private bool IsCollision(Vector3 point)
        {
            if (point.Y < -0.35f) return true;
            if (point.Y > 0.1f) return true;

            return false;
        }


        public List<Vector3> PlanAndExecutePath(Vector3 target, Vector3 current)
        {
            List<Vector3> joints = new();
            int steps = 10;  // More = smoother
            for (int i = 1; i <= steps; i++)
            {
                Vector3 interp = Vector3.Lerp(current, target, i / (float)steps);

                if (IsCollision(interp))
                {
                    Console.WriteLine($"Path blocked at step {i} at {interp}");
                    break;  // Stop planning
                }

                int idx = kdTree.FindNearest(new float[] { interp.X, interp.Y, interp.Z});
                Vector3 joint = positionData[idx].joint;
                joints.Add(joint);

                
            }

            return joints;
        }
        

        



        public async Task LoadCSV()
        {
            string path = "/home/orangepi/grid/handGrid.csv";

            if (!File.Exists(path))
            {
                Console.WriteLine("File not found: " + path);
                return;
            }

            string[] lines = File.ReadAllLines(path);
            int i = 0;

            for (int lineIndex = 1; lineIndex < lines.Length; lineIndex++) // Skip header
            {
                string line = lines[lineIndex];
                if (string.IsNullOrWhiteSpace(line)) continue;

                var parts = line.Split(';');
                if (parts.Length < 8) continue;

                PositionEntry entry = new PositionEntry
                {
                    time = float.Parse(parts[0], CultureInfo.CurrentCulture),
                    objectName = parts[1],
                    position = new Vector3(
                        float.Parse(parts[2], CultureInfo.CurrentCulture),
                        float.Parse(parts[3], CultureInfo.CurrentCulture),
                        float.Parse(parts[4], CultureInfo.CurrentCulture)),

                    joint = new Vector3(
                        float.Parse(parts[5], CultureInfo.CurrentCulture),
                        float.Parse(parts[6], CultureInfo.CurrentCulture),
                        float.Parse(parts[7], CultureInfo.CurrentCulture))
                };

                positionData.Add(entry);
                kdTree.AddPoint(new float[] { entry.position.X, entry.position.Y, entry.position.Z }, i);
                i++;
                //if (i % 10000 == 0) Console.WriteLine(entry.position.X + " " + entry.position.Y + " " + entry.position.Z);

            }

            Console.WriteLine("Finished Loading Data, number of lines: " + i + " " + positionData.Count + " " + kdTree.ToString());
            await Task.Delay(15);
            await Task.Yield();
        }
       
    }

    public class KDTree<T>
    {
        private class Node
        {
            public float[] point;
            public T value;
            public Node left;
            public Node right;

            public Node(float[] point, T value)
            {
                this.point = point;
                this.value = value;
            }
        }

        private Node root;

        public void AddPoint(float[] point, T value)
        {
            if (point.Length != 3)
                throw new ArgumentException("Only 3D points are supported.");
            root = Insert(root, point, value, 0);
        }

        private Node Insert(Node node, float[] point, T value, int depth)
        {
            if (node == null)
                return new Node(point, value);

            int axis = depth % 3;
            if (point[axis] < node.point[axis])
                node.left = Insert(node.left, point, value, depth + 1);
            else
                node.right = Insert(node.right, point, value, depth + 1);

            return node;
        }

        public T FindNearest(float[] target)
        {
            if (target.Length != 3)
                throw new ArgumentException("Only 3D target points supported.");
            return FindNearest(root, target, 0, root, float.MaxValue).node.value;
        }

        private (Node node, float dist) FindNearest(Node current, float[] target, int depth, Node best, float bestDist)
        {
            if (current == null)
                return (best, bestDist);

            float dist = DistanceSquared(current.point, target);
            if (dist < bestDist)
            {
                best = current;
                bestDist = dist;
            }

            int axis = depth % 3;
            Node near = target[axis] < current.point[axis] ? current.left : current.right;
            Node far = near == current.left ? current.right : current.left;

            (best, bestDist) = FindNearest(near, target, depth + 1, best, bestDist);

            if ((target[axis] - current.point[axis]) * (target[axis] - current.point[axis]) < bestDist)
            {
                (best, bestDist) = FindNearest(far, target, depth + 1, best, bestDist);
            }
            return (best, bestDist);
        }

        public List<T> FindKNearest(float[] target, int k)
        {
            var resultHeap = new SortedList<float, T>(new DuplicateKeyComparer<float>());
            FindKNearest(root, target, 0, k, resultHeap);
            return resultHeap.Values.ToList();
        }

        private void FindKNearest(Node current, float[] target, int depth, int k, SortedList<float, T> heap)
        {
            if (current == null) return;

            float dist = DistanceSquared(current.point, target);

            if (heap.Count < k)
            {
                heap.Add(dist, current.value);
            }
            else if (dist < heap.Keys[heap.Count - 1])
            {
                heap.RemoveAt(heap.Count - 1);
                heap.Add(dist, current.value);
            }

            int axis = depth % 3;
            bool goLeft = target[axis] < current.point[axis];
            Node near = goLeft ? current.left : current.right;
            Node far = goLeft ? current.right : current.left;

            FindKNearest(near, target, depth + 1, k, heap);

            float axisDistSq = (target[axis] - current.point[axis]) * (target[axis] - current.point[axis]);
            if (heap.Count < k || axisDistSq < heap.Keys[heap.Count - 1])
            {
                FindKNearest(far, target, depth + 1, k, heap);
            }
        }

        private float DistanceSquared(float[] a, float[] b)
        {
            float dx = a[0] - b[0];
            float dy = a[1] - b[1];
            float dz = a[2] - b[2];
            return dx * dx + dy * dy + dz * dz;
        }
    }

    public class DuplicateKeyComparer<TKey> : IComparer<TKey> where TKey : IComparable
    {
        public int Compare(TKey x, TKey y)
        {
            int result = x.CompareTo(y);
            return result == 0 ? 1 : result; // Allow duplicate keys
        }
    }
}
