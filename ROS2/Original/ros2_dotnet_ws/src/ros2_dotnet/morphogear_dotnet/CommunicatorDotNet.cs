using System;
using ROS2;
using System.Collections.Generic;
using System.Threading.Tasks;
using System.Threading.Tasks.Dataflow;
using builtin_interfaces.msg;
using System.Threading;
using System.Diagnostics;
using std_msgs.msg;



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

            controlAngles = new List<sbyte>{0,0,0,0,0,0,0,0,0,0,0,0};
            stateAngles = new List<sbyte>{0,0,0,0,0,0,0,0,0,0,0,0};
            SetAngleDelegate = new SetAngle(SetControlAngle);
            GetAngleDelegate = new GetAngle(GetStateAngle);
            Request_KMPC_Delegate = new Request_KMPC(Request_KMPC_Angles);

            Subscription<std_msgs.msg.String> CMDSub = node.CreateSubscription<std_msgs.msg.String>(
                "morphogear_sudo_cmd",CMD_callback);

            Subscription<std_msgs.msg.Bool> ManualSub = node.CreateSubscription<std_msgs.msg.Bool>(
                "morphogear_sudo_manual",Manual_callback);

            Subscription<std_msgs.msg.Int8MultiArray> chatterSub = node.CreateSubscription<std_msgs.msg.Int8MultiArray>(
                "/angles_control",Angles_callback);

            Subscription<std_msgs.msg.Int8MultiArray> KMPCSub = node.CreateSubscription<std_msgs.msg.Int8MultiArray>(
                "/theta_angles" , KMPC_callback);


        
            _KMPCPub = node.CreatePublisher<std_msgs.msg.Empty>("/request_angles");

            _anglePub = node.CreatePublisher<std_msgs.msg.Int8MultiArray>("/angles_control");

            node.CreateTimer(TimeSpan.FromSeconds(0.05), PublishAngles);        

        }

        /// <summary>
        /// Angles Properties
        /// </summary>
        private List<sbyte> controlAngles;
        private List<sbyte> stateAngles;

        private bool controlType=false;

        private List<sbyte> KMPCAngles;
        private Subscription <std_msgs.msg.Int8MultiArray> chatterSub;
        private readonly Publisher<std_msgs.msg.Empty> _KMPCPub;
        private readonly Publisher<std_msgs.msg.Int8MultiArray> _anglePub;
        private readonly std_msgs.msg.Int8MultiArray _anglePubMsg= new();

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
                controlAngles=msg.Data;
                
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
            controlAngles[index]=value;
        }

    }
    


    public class Control
    {
        private static Dictionary<string,Action<int>> dict = new Dictionary<string, Action<int>>();
        private bool _controlRights = true;


        private int initialStandingAngle=30;

        Mover[,] motors = new Mover[4,3];
        
        public Control()
        {
            for (int limb =0;limb<4;limb++)
                motors[limb,0]= new Mover(-30,30,limb); //Shoulder
            for (int limb =0;limb<4;limb++)
                motors[limb,1]= new Mover(-90, 30 ,limb+4); //UpperArm
            for (int limb =0;limb<4;limb++)
                motors[limb,2]= new Mover(-90, 90 ,limb+8); //ForeArm
            


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
            dict.Add("forwardKMPC", (x) => MoveRobotByKMPC(forward,x)); 
            dict.Add("rightKMPC", (x) => MoveRobotByKMPC(right,x));  
            dict.Add("backwardKMPC", (x) => MoveRobotByKMPC(backward,x));  
            dict.Add("leftKMPC", (x) => MoveRobotByKMPC(left,x));  

                
        }

        public void ExecuteCMD(string cmd)
        {
            var command=cmd.Split("_");
            int value = 30;
            if (command.Length>1)
            {
                value = int.Parse(command[1]);
                Console.WriteLine(command[0]+" " + command[1]);
            }
            else
            {
                command[0]=cmd;
            }
            if (_controlRights)
                dict[command[0]](value);
            else
                Console.WriteLine("wait for previous execution");
        }

        public async void MoveRobotBySequence(List<sbyte> angles)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Walking KMPC Trajectories");
            await Task.Delay(100);
            if (CommunicatorDotNet.GetAngleDelegate(0)!=0||CommunicatorDotNet.GetAngleDelegate(1)!=0||CommunicatorDotNet.GetAngleDelegate(2)!=0||CommunicatorDotNet.GetAngleDelegate(3)!=0)
                await _InitialPosition(token);
            var task =_ExecuteTrajectory(angles, token);
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
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            await Task.Delay(100);
            if (((CommunicatorDotNet.GetAngleDelegate(direction-1)!=0||CommunicatorDotNet.GetAngleDelegate((-1+direction +2)%4)!=0)))
                {//Console.WriteLine("1");
                await _InitialPosition(token);}
            if ((CommunicatorDotNet.GetAngleDelegate(direction+4-1)!=-initialStandingAngle||CommunicatorDotNet.GetAngleDelegate((-1+direction +2)%4+4)!=-initialStandingAngle)) 
                {//Console.WriteLine("2");
                await _InitialPosition(token);}
            if ((CommunicatorDotNet.GetAngleDelegate(-1+direction+8)!=-90+initialStandingAngle||CommunicatorDotNet.GetAngleDelegate((-1+direction +2)%4+8)!=-90+initialStandingAngle))
                {//Console.WriteLine("3");
                await _InitialPosition(token);}

            await Task.Delay(100);
            Console.WriteLine("Walking KMPC Gait 1 Trajectory");
            var task =_Gait1Walk(direction, token, angle);
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
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;


            Console.WriteLine("Initial Position");
            var task =_InitialPosition(token);
            var delayTask = Task.Delay(20000); //timeout
            await Task.WhenAny(task, delayTask);
            cts.Cancel();

            cts = null;
            token = CancellationToken.None; //CLEAR VARIABLES

            //await Task.Run(_InitialPosition);

            Console.WriteLine("Initial Position executed");
            _controlRights = true;
        }


        private async void Gait1Walk(int direction)
        {
            CancellationTokenSource cts = new CancellationTokenSource();
            CancellationToken token = cts.Token;
            _controlRights = false;
            Console.WriteLine("Walking with Gait 1");
            if (CommunicatorDotNet.GetAngleDelegate((direction+1)%4)!=0||CommunicatorDotNet.GetAngleDelegate((direction+3)%4)!=0)
                await _InitialPosition(token);
            var task =_Gait1Walk(direction, token);
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

            var task =_Gait2Walk(direction, token);
            var delayTask = Task.Delay(20000);
            await Task.WhenAny(task, delayTask); //timeout
            cts.Cancel();

            cts = null;
            token = CancellationToken.None;

            //await Task.Run(()=>_Gait1Walk(direction));
            Console.WriteLine("Walking with Gait 2 executed");
            _controlRights = true;
        }

        private async Task _InitialPosition(CancellationToken token)
        {

            var angle1 = - initialStandingAngle;
            var angle2 = -90 + initialStandingAngle;
            await Task.Delay(10);
            if (Math.Abs(motors[0,0].CurrentPrimaryAxisRotation()) > 3)
            {
                motors[0,1].RotateTarget(0,token);
                await motors[0,2].RotateTarget(-90,token);
                await motors[0,0].RotateTarget(0,token);
                motors[0,1].RotateTarget(angle1,token);
                await motors[0,2].RotateTarget(angle2,token);
            }

            motors[0,0].RotateTarget(0,token);
            motors[0,1].RotateTarget(angle1,token);
            await motors[0,2].RotateTarget(angle2,token);//;
            

            if (Math.Abs(motors[2,0].CurrentPrimaryAxisRotation()) > 3)
            {
                motors[2,1].RotateTarget(0,token);
                await motors[2,2].RotateTarget(-90,token);
                await motors[2,0].RotateTarget(0,token);
                motors[2,1].RotateTarget(angle1,token);
                await motors[2,2].RotateTarget(angle2,token);
            }

            motors[2,0].RotateTarget(0,token);
            motors[2,1].RotateTarget(angle1,token);
            await motors[2,2].RotateTarget(angle2,token);//;


        if (Math.Abs(motors[3,0].CurrentPrimaryAxisRotation()) > 3)
        {
            motors[3,1].RotateTarget(0,token);
            await motors[3,2].RotateTarget(-90,token);
            await motors[3,0].RotateTarget(0,token);
            motors[3,1].RotateTarget(angle1,token);
            await motors[3,2].RotateTarget(angle2,token);//;
        }


        motors[3,0].RotateTarget(0,token);
        motors[3,1].RotateTarget(angle1,token);//;
        await motors[3,2].RotateTarget(angle2,token);//;
        
        
        if (Math.Abs(motors[1,0].CurrentPrimaryAxisRotation()) > 3)
        {
            motors[1,1].RotateTarget(0,token);
            await motors[1,2].RotateTarget(-90,token);
            await motors[1,0].RotateTarget(0,token);
            motors[1,1].RotateTarget(angle1,token);
            await motors[1,2].RotateTarget(angle2,token);//;
        }

        motors[1,0].RotateTarget(0,token);
        motors[1,1].RotateTarget(angle1,token);
        await motors[1,2].RotateTarget(angle2,token);//;

        await Task.Yield();
        }


        private async Task _Gait1Walk(int direction,CancellationToken token, int angle = 30)
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

            Halfstep(right, true,token, angle);
            await Halfstep(left, false,token, angle);
            await Task.Delay(10);
            await Stepper(right, left, back, forward,token, angle);


                motors[back,0].RotateTarget(0,token);
                motors[back,1].RotateTarget(-initialStandingAngle,token);
               await motors[back,2].RotateTarget(-90 + initialStandingAngle,token);

               await Task.Delay(10);

                motors[forward,0].RotateTarget(0,token);
                motors[forward,1].RotateTarget(-initialStandingAngle,token);
             await   motors[forward,2].RotateTarget(-90 + initialStandingAngle,token);

             await Task.Delay(10);
             await Task.Yield();
        }

        private async Task _Gait2Walk(int direction,CancellationToken token)
        {
            token.ThrowIfCancellationRequested();

            await _InitialPosition(token); //Initial walk position

            await Walk(direction,token);
        }

        private async Task _ExecuteTrajectory(List<sbyte> angles, CancellationToken token)
        {

             for (int i=0;i<200;i++)
            {    
                          
                if (token.IsCancellationRequested)
                            return;
                motors[0,0].RotateTo(0);
                motors[1,0].RotateTo(0);
                motors[2,0].RotateTo(0);
                motors[3,0].RotateTo(0);

                
                motors[0,1].RotateTo(angles[i]);
                motors[0,2].RotateTo(angles[200+i]);
                motors[1,1].RotateTo(angles[400+i]);
                motors[1,2].RotateTo(angles[600+i]);
                motors[2,1].RotateTo(angles[800+i]);
                motors[2,2].RotateTo(angles[1000+i]);
                motors[3,1].RotateTo(angles[1200+i]);
                motors[3,2].RotateTo(angles[1400+i]);
           
                await Task.Delay(20);
                await Task.Yield();
            }
        }
        
            private async Task Walk(int direction,  CancellationToken token)
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

                motors[_rightForward,1].RotateTo((sbyte)angles[0][i]);//, token); // Start loop
                motors[_rightForward,2].RotateTo((sbyte)angles[1][i]);//, token);
                motors[_leftForward,1].RotateTo((sbyte)angles[0][(i + trajectoryHalfPointNumber) % (2 * trajectoryHalfPointNumber)]);//, token); //Start from half loop
                motors[_leftForward,2].RotateTo((sbyte)angles[1][(i + trajectoryHalfPointNumber) % (2 * trajectoryHalfPointNumber)]);//, token);
                motors[_rightHind,1].RotateTo((sbyte)angles[0][2 * trajectoryHalfPointNumber - 1 - i]);//, token); //Start backloop
                motors[_rightHind,2].RotateTo((sbyte)angles[1][2 * trajectoryHalfPointNumber - 1 - i]);//, token);
                motors[_leftHind,1].RotateTo((sbyte)angles[0][(3 * trajectoryHalfPointNumber - i) % (2 * trajectoryHalfPointNumber)]);//, token);
                motors[_leftHind,2].RotateTo((sbyte)angles[1][(3 * trajectoryHalfPointNumber - i) % (2 * trajectoryHalfPointNumber)]);//, token);
                

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
            

                i = i%(2 * trajectoryHalfPointNumber);

                await Task.Delay(10);
                await Task.Yield();
            }
        }

        public float[][] trajectoryGenerator(int segment_num, float multiplicator = 0.8f, float _archimed_compression = 0.7f)
        {
            var _robotStandingHeight = (float)Math.Sin(initialStandingAngle * Math.PI / 180) * L_UA + L_FA -10;
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




        private async Task Halfstep(int leg, bool side, CancellationToken token, int angle)
            {
                motors[leg,1].RotateTarget(0,token);
                await  motors[leg,2].RotateTarget(-90,token);

                if (side)
                    await  motors[leg,0].RotateTarget(angle, token);
                else
                    await  motors[leg,0].RotateTarget(-angle, token);

                motors[leg,1].RotateTarget(-initialStandingAngle,token);
                await  motors[leg,2].RotateTarget(-90 + initialStandingAngle,token);
            }
            internal int L_UA = 155;
            internal int L_FA = 275;
            internal int L_Sh = 95;

        private async Task Stepper(int right, int left, int back, int forward, CancellationToken token, int angle)
        {
            var speedRotation = 1;
            var upAngle = 10;
            
            float targetAngle = angle;
            float initialRotationright = motors[right,0].CurrentPrimaryAxisRotation();
            float initialRotationleft = motors[left,0].CurrentPrimaryAxisRotation();
            var loop = true;
            float deltaAngle = targetAngle - initialRotationleft;
            float rotationTotal;
            float rotation;
            float l_max = (float)(L_UA * Math.Cos(initialStandingAngle * Math.PI /180) + L_Sh);  //Projection on the floor L_shoulder+L_FA*cos(theta1^init)

            motors[left,0].rotationState = 1;
            motors[right,0].rotationState = -1;

            while (loop)
            {
                if (token.IsCancellationRequested)
                            return;


                float rotationChange = (float)motors[left,0].rotationState * speedRotation; //*dt
                float rotationGoalleft = motors[left,0].CurrentPrimaryAxisRotation() + rotationChange;
                float rotationGoalright = motors[right,0].CurrentPrimaryAxisRotation() - rotationChange;
                rotationTotal = Math.Abs(initialRotationleft - rotationGoalleft);


                if (rotationTotal < targetAngle)
                    rotation = rotationTotal;
                else
                    rotation = 2 * targetAngle - rotationTotal;

                //FROM ARTICLES with mistake* - length of the shoulder joint wasn't considered
                //float rotationGoalHeight = Mathf.Acos((2 * Mathf.Cos(rotation * Mathf.Deg2Rad) - 1) * Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad)) * Mathf.Rad2Deg - initialStandingAngle;

                float rotationGoalHeight = (float)(Math.Acos(2 *(Math.Cos(rotation * Math.PI /180) - 1) * l_max / L_UA + Math.Cos(initialStandingAngle * Math.PI /180)) * 180/Math.PI - initialStandingAngle);


                if ((rotationGoalleft < motors[left,0].lowerLimit) | (rotationGoalleft > motors[left,0].upperLimit)) //Check limitations
                    return;

                
                if (rotationTotal > Math.Abs(deltaAngle))  //Fixing angles
                {
                    loop = false;
                    motors[right,0].RotateTo((sbyte)(initialRotationright - deltaAngle));
                    motors[left,0].RotateTo((sbyte)(initialRotationleft + deltaAngle));
                    motors[right,1].RotateTo((sbyte)(-initialStandingAngle));
                    motors[right,2].RotateTo((sbyte)(-90 + initialStandingAngle));
                    motors[left,1].RotateTo((sbyte)(-initialStandingAngle));
                    motors[left,2].RotateTo((sbyte)(-90 + initialStandingAngle));


                    await motors[back,2].RotateTarget((sbyte)(-90 + initialStandingAngle),token);
                    motors[back,1].RotateTarget((sbyte)(-initialStandingAngle),token);

                    //forward[1].RotateTo(-initialStandingAngle + upAngle);


                    motors[forward,2].RotateTarget((sbyte)(-90 + initialStandingAngle),token);
                    await motors[forward,1].RotateTarget((sbyte)(-initialStandingAngle),token);

                    return;
                }
                //else
                

                motors[right,0].RotateTo((sbyte)(rotationGoalright));
                motors[left,0].RotateTo((sbyte)(rotationGoalleft));
                motors[right,1].RotateTo((sbyte)(-initialStandingAngle - rotationGoalHeight));
                motors[right,2].RotateTo((sbyte)(-90 + initialStandingAngle + rotationGoalHeight));
                motors[left,1].RotateTo((sbyte)(-initialStandingAngle - rotationGoalHeight));
                motors[left,2].RotateTo((sbyte)(-90 + initialStandingAngle + rotationGoalHeight));

                if (rotationGoalleft < 0)
                     motors[back,2].RotateTarget((sbyte)(-90 + initialStandingAngle + upAngle * rotationTotal / deltaAngle),token);
                else
                     motors[back,1].RotateTo((sbyte)(-initialStandingAngle + 0.5f * rotationGoalleft / targetAngle * upAngle));   //upAngle * rotationTotal / deltaAngle);



                motors[forward,1].RotateTo((sbyte)(-initialStandingAngle + upAngle * rotationTotal / deltaAngle));// +rotationGoalHeight);
                motors[forward,2].RotateTo((sbyte)(-90 + initialStandingAngle - 0.5f * upAngle * rotationTotal / deltaAngle));


                await Task.Delay(15);
                await Task.Yield();
            }
    
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
                    return;}


                if (rotationTotal > Math.Abs(deltaAngle))
                {
                    
                    var final=(sbyte)(initialRotation + deltaAngle);
                    await Task.Delay(15);
                    //Console.WriteLine(rotationGoal + "  2");
                    RotateTo(final);
                    return;
                }
                else
                    RotateTo((sbyte)rotationGoal);
                
                await Task.Delay(15);
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
            primaryAxisRotation=(sbyte)Math.Clamp(primaryAxisRotation,lowerLimit,upperLimit);
            CommunicatorDotNet.SetAngleDelegate(primaryAxisRotation, index);
        }
   
    }



}
