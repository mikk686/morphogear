using Calculations;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UIElements;

public class ROSTelemetry : MonoBehaviour
{

    ROSConnection ros;
    public string pubTopicName = "angles_control";
    public string subTopicName = "angles_control";

    public string topicName = "/vicon/morphogear";

    public string depthTopic = "/depth";

    public string odometryTopic = "/odometry";


    public GameObject morphogear;



    public bool mirror;
    public bool simulation;

    public bool visualization;


    private ArticulationBody articulationbody;
    private Control control;
    private int dofCount;

    //public float publishMessageFrequency = 0.05f;


    public List<float> positions = new List<float>(12);
    private List<float> _controlPositions = new List<float>(18);
    // Start is called once before the first execution of Update after the MonoBehaviour is created


    public RGBDCamera rgbdCamera;
    private byte[] _depth16UC1;
    private bool isPublishingDepth = false;

    private void Awake()
    {
        for (int i = 0; i < 12; ++i)
        {
            positions.Add(0f);
            _controlPositions.Add(0f);
        }

        _depth16UC1 = new byte[640 * 480 * 2];
    }



    void Start()
    {
        articulationbody = morphogear.GetComponent<ArticulationBody>();
        control = morphogear.GetComponent<Control>();

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Int8MultiArrayMsg>(pubTopicName);  //if mirroring to robot
        ros.Subscribe<Int8MultiArrayMsg>(subTopicName, JointCallbackSmooth); //if visualization of robot state
        ros.RegisterPublisher<TransformStampedMsg>(topicName); //if simulation for robot 

        ros.RegisterPublisher<ImageMsg>(depthTopic);

        if (rgbdCamera != null)
        {
            rgbdCamera.OnSensorUpdated += OnDepthUpdated;
        }

        ros.RegisterPublisher<OdometryMsg>(odometryTopic);
    }

    // Update is called once per frame
    void Update()
    {

        

    }


    void FixedUpdate()
    {

        if (mirror) ros.Publish(pubTopicName, UpdateIntJointsStateStd());

        if (simulation) ros.Publish(topicName, UpdateTransform());

        PublishOdometry();


    }

    private void PublishOdometry()
    {
        // 1. Позиция и поворот в мировых координатах Unity (предполагаем, что старт в 0,0,0)
        // Если старт не в 0,0,0, нужно вычитать стартовый трансформ!
        Vector3 worldPos = morphogear.transform.position;
        Quaternion worldRot = morphogear.transform.rotation;

        // 2. ПЕРЕВОД СКОРОСТЕЙ ИЗ ГЛОБАЛЬНЫХ В ЛОКАЛЬНЫЕ (Body Frame)
        // InverseTransformDirection переводит вектор из мирового пространства Unity в локальное пространство morphogear
        Vector3 localLinearVel = morphogear.transform.InverseTransformDirection(articulationbody.linearVelocity);

        // Для угловой скорости в Unity ArticulationBody использует мировые координаты, переводим в локальные:
        Vector3 localAngularVel = morphogear.transform.InverseTransformDirection(articulationbody.angularVelocity);

        OdometryMsg odomMsg = new OdometryMsg
        {
            // В ROS2/ROS обычно не используют ведущий слэш "/" для frame_id (пишут просто "odom" и "base_link")
            header = new HeaderMsg { frame_id = "odom", stamp = GetCurrentRosTime() },
            child_frame_id = "base_link",

            // Pose выражается в ГЛОБАЛЬНЫХ координатах (odom) -> переводим глобальный Unity в FLU
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = worldPos.To<FLU>(),
                    orientation = worldRot.To<FLU>()
                }
            },

            // Twist выражается в ЛОКАЛЬНЫХ координатах (base_link) -> переводим локальный Unity в FLU
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = localLinearVel.To<FLU>(),
                    angular = localAngularVel.To<FLU>()
                }
            }
        };

        ros.Publish(odometryTopic, odomMsg);
    }


    private async void OnDepthUpdated()
    {
        if (isPublishingDepth || !rgbdCamera.Depth16Data.IsCreated) return;
        isPublishingDepth = true;
        try
        {
            await Task.Run(() => PackDepthDataUltraFast(rgbdCamera.Depth16Data, _depth16UC1));

            ImageMsg depthMsg = new ImageMsg
            {
                header = new HeaderMsg { frame_id = $"camera_depth_link", stamp = GetCurrentRosTime() },
                height = (uint)480,
                width = (uint)640,
                encoding = "16UC1",
                is_bigendian = 0,
                step = (uint)(640 * 2),
                data = _depth16UC1
            };

            ros.Publish(depthTopic, depthMsg);
        }
        finally
        {
            isPublishingDepth = false;
        }
    }

    private void PackDepthDataUltraFast(NativeArray<uint> gpuDepthData, byte[] destinationByteArray)
    {
        int length = gpuDepthData.Length;
        unsafe
        {
            fixed (byte* destPtr = destinationByteArray)
            {
                ushort* dest16 = (ushort*)destPtr;
                for (int i = 0; i < length; i++)
                {
                    dest16[i] = (ushort)gpuDepthData[i];
                }
            }
        }
    }

    private TimeMsg GetCurrentRosTime()
    {
        int sec = (int)Time.realtimeSinceStartup;
        uint nanosec = (uint)((Time.realtimeSinceStartup - sec) * 1e9);
        return new TimeMsg(sec, nanosec);
    }

    private Int8MultiArrayMsg UpdateIntJointsStateStd()
    {
        articulationbody.GetJointPositions(positions);
        if (articulationbody.immovable) dofCount = 0;
        else dofCount = 6;

        sbyte lf_shoulder = Convert.ToSByte(positions[0 + dofCount] * Mathf.Rad2Deg);
        sbyte rf_shoulder = Convert.ToSByte(positions[1 + dofCount] * Mathf.Rad2Deg);
        sbyte lh_shoulder = Convert.ToSByte(positions[2 + dofCount] * Mathf.Rad2Deg);
        sbyte rh_shoulder = Convert.ToSByte(positions[3 + dofCount] * Mathf.Rad2Deg);
        sbyte lf_upperarm = Convert.ToSByte(positions[4 + dofCount] * Mathf.Rad2Deg);
        sbyte rf_upperarm = Convert.ToSByte(positions[5 + dofCount] * Mathf.Rad2Deg);
        sbyte lh_upperarm = Convert.ToSByte(positions[6 + dofCount] * Mathf.Rad2Deg);
        sbyte rh_upperarm = Convert.ToSByte(positions[7 + dofCount] * Mathf.Rad2Deg);
        sbyte lf_forearm = Convert.ToSByte(positions[8 + dofCount] * Mathf.Rad2Deg);
        sbyte rf_forearm = Convert.ToSByte(positions[9 + dofCount] * Mathf.Rad2Deg);
        sbyte lh_forearm = Convert.ToSByte(positions[10 + dofCount] * Mathf.Rad2Deg);
        sbyte rh_forearm = Convert.ToSByte(positions[11 + dofCount] * Mathf.Rad2Deg);


        var data = new sbyte[] { rf_shoulder, rh_shoulder, lh_shoulder, lf_shoulder, rf_upperarm, rh_upperarm, lh_upperarm, lf_upperarm, rf_forearm, rh_forearm, lh_forearm, lf_forearm };


        Int8MultiArrayMsg messege = new Int8MultiArrayMsg();
        messege.data = data;

        // Properly initialize the layout
        messege.layout = new MultiArrayLayoutMsg();
        messege.layout.data_offset = 0;
        messege.layout.dim = new MultiArrayDimensionMsg[1];
        messege.layout.dim[0] = new MultiArrayDimensionMsg();
        messege.layout.dim[0].label = "joints";
        messege.layout.dim[0].size = (uint)data.Length;
        messege.layout.dim[0].stride = (uint)data.Length;

        return messege;
    }


    void JointCallbackSmooth(Int8MultiArrayMsg msg)
    {

        string numbers = string.Join(", ", msg.data);

        //Debug.Log($"Receive elements count: {msg.data.Length}. Data: [{numbers}]");
        if (!mirror) _ = control.MoveRobotByThetas(msg.data);


    }



    private TransformStampedMsg UpdateTransform()
    {
        TransformStampedMsg message = new TransformStampedMsg(
            UpdateHeader(),
            "map",  // child_frame_id - replace with actual frame name if needed
            UpdatePosition()
        );
        return message;
    }

    private TransformMsg UpdatePosition()
    {
        return new TransformMsg(V3msg(), QuatMsg());
    }

    private Vector3Msg V3msg()
    {
        Vector3<FLU> rosPose = morphogear.transform.position.To<FLU>();
        return new Vector3Msg(rosPose.x, rosPose.y, rosPose.z);
    }

    private QuaternionMsg QuatMsg()
    {
        return morphogear.transform.rotation.To<FLU>();
    }

    private HeaderMsg UpdateHeader()
    {
        return new HeaderMsg(UpdateTime(), "map");
    }

    private TimeMsg UpdateTime()
    {
        var timeSinceEpoch = DateTime.UtcNow - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
        int sec = (int)timeSinceEpoch.TotalSeconds;
        uint nanosec = (uint)((timeSinceEpoch.TotalSeconds - sec) * 1e9);

        return new TimeMsg(sec, nanosec);
    }
}
