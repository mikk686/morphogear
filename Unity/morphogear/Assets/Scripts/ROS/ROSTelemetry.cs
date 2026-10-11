using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;
using System.Collections.Generic;
using System.IO;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.UI;
using static Unity.VectorGraphics.VectorUtils;

public enum PoseChannelMode
{
    Off,
    Publish,
    Receive
}

public enum TeleportPoseSource
{
    PoseTransform,
    ReceivedPoseStamped
}

public class ROSTelemetry : MonoBehaviour
{
    [Header("Robot")]
    public GameObject morphogear;
    public UnifiedKeyboardController keyboardController;

    [Header("Joints")]
    public string pubTopicName = "angles_control";
    public string subTopicName = "angles_control_mg";

    [Header("PoseStamped")]
    public string poseTopic = "/odometry";
    public PoseChannelMode poseChannelMode = PoseChannelMode.Off;

    [Header("Robot visualization")]
    public bool visualization;
    public TeleportPoseSource teleportPoseSource = TeleportPoseSource.PoseTransform;
    public Transform poseTransform;

    [Header("Depth")]
    public string depthTopic = "/depth";
    public RGBDCamera rgbdCamera;
    public bool publishDepth;

    private ROSConnection ros;
    private ArticulationBody articulationbody;


    public string odomTopic = "odometry";
    private Control control;

    public List<float> positions = new List<float>(18);

    private byte[] _depth16UC1;
    private bool isPublishingDepth;

    private readonly object poseLock = new object();
    private Vector3 receivedPosition;
    private Quaternion receivedRotation;
    private bool hasReceivedPose;

    private bool changedImmovable;
    private bool previousImmovable;

    [Header("Joint routing — controlled by GameManager")]
    public bool publishJoints;
    public bool receiveJoints;

    //private bool RobotMode =>
    //    keyboardController != null &&
    //    keyboardController.CurrentDestination == KeyboardDestination.Robot;

    private void Awake()
    {
        for (int i = 0; i < 18; i++)
            positions.Add(0f);

        _depth16UC1 = new byte[640 * 480 * 2];
        InitializeIfNeeded();
    }
    private bool initialized;

    //private void OnEnable()
    //{
    //    InitializeIfNeeded();

    //    if (rgbdCamera != null)
    //        rgbdCamera.OnSensorUpdated += OnDepthUpdated;
    //}


    private void OnEnable()
    {
        nextJointPublishTime = 0;
        nextPosePublishTime = 0;
        nextDepthPublishTime = 0;

        InitializeIfNeeded();

        if (rgbdCamera != null)
            rgbdCamera.OnSensorUpdated += OnDepthUpdated;
    }
    private void OnDisable()
    {
        if (rgbdCamera != null)
            rgbdCamera.OnSensorUpdated -= OnDepthUpdated;

        RestoreImmovable();
    }

    private void InitializeIfNeeded()
    {
        if (initialized)
            return;

        articulationbody =
            morphogear.GetComponent<ArticulationBody>();

        control =
            morphogear.GetComponent<Control>();

        ros = ROSConnection.GetOrCreateInstance();


        



        ros.RegisterPublisher<Int8MultiArrayMsg>(pubTopicName);
        ros.RegisterPublisher<PoseStampedMsg>(poseTopic);


        //ros.Subscribe<Int8MultiArrayMsg>(subTopicName, JointCallbackSmooth);
        //ros.Subscribe<PoseStampedMsg>(poseTopic, PoseCallback);




        StartCoroutine(DelayedSubscribeRoutine());

        ros.RegisterPublisher<ImageMsg>(
            depthTopic);

        ros.RegisterPublisher<OdometryMsg>(odomTopic);

        initialized = true;
    }


    private System.Collections.IEnumerator DelayedSubscribeRoutine()
    {
        // Wait for the end of the frame or next frame 
        // to guarantee the Publisher packet clears the network buffer
        yield return new WaitForEndOfFrame();
        yield return null;

        ros.Subscribe<PoseStampedMsg>(poseTopic, PoseCallback);
        ros.Subscribe<Int8MultiArrayMsg>( subTopicName, JointCallbackSmooth);
    }
    //private void Start()
    //{
    //    articulationbody = morphogear.GetComponent<ArticulationBody>();
    //    control = morphogear.GetComponent<Control>();

    //    //if (keyboardController == null)
    //    //    keyboardController =
    //    //        morphogear.GetComponent<UnifiedKeyboardController>();

    //    if (keyboardController == null)
    //    {
    //        Debug.LogError(
    //            "ROSTelemetry needs a UnifiedKeyboardController reference.",
    //            this);
    //        enabled = false;
    //        return;
    //    }

    //    ros = ROSConnection.GetOrCreateInstance();

    //    ros.RegisterPublisher<Int8MultiArrayMsg>(pubTopicName);
    //    ros.Subscribe<Int8MultiArrayMsg>(subTopicName, JointCallbackSmooth);

    //    ros.RegisterPublisher<PoseStampedMsg>(poseTopic);
    //    ros.Subscribe<PoseStampedMsg>(poseTopic, PoseCallback);

    //    ros.RegisterPublisher<ImageMsg>(depthTopic);

    //    if (rgbdCamera != null)
    //        rgbdCamera.OnSensorUpdated += OnDepthUpdated;
    //}

    //private void OnDestroy()
    //{
    //    if (rgbdCamera != null)
    //        rgbdCamera.OnSensorUpdated -= OnDepthUpdated;

    //    RestoreImmovable();
    //}

    //private void FixedUpdate()
    //{
    //    // Visualization can make the root immovable. Restore its previous
    //    // state before returning to Unity control and reading joint positions.
    //    bool teleportRobot =
    //        RobotMode &&
    //        visualization &&
    //        poseChannelMode != PoseChannelMode.Publish;

    //    if (teleportRobot)
    //    {
    //        if (!changedImmovable)
    //        {
    //            previousImmovable = articulationbody.immovable;
    //            changedImmovable = true;
    //        }

    //        PoseTeleport();
    //    }
    //    else
    //    {
    //        RestoreImmovable();
    //    }

    //    // Exactly one joint-data direction is active.
    //    if (!RobotMode)
    //        ros.Publish(pubTopicName, UpdateIntJointsStateStd());

    //    // Independent of joint mode. Publish and Receive are mutually
    //    // exclusive, preventing this component from reading its own poses.
    //    if (poseChannelMode == PoseChannelMode.Publish)
    //        PublishPose();
    //}

    private double nextJointPublishTime;
    private double nextPosePublishTime;

    private void FixedUpdate()
    {



        if (ros == null || articulationbody == null)
            return;


        

        if (visualization)
        {
            if (!changedImmovable)
            {
                previousImmovable = articulationbody.immovable;
                changedImmovable = true;
            }

            PoseTeleport();
        }
        else
        {
            RestoreImmovable();
        }

        double now = Time.realtimeSinceStartupAsDouble;

        if (publishJoints && now >= nextJointPublishTime)
        {
            // Schedule from NOW. Do not send extra messages to catch up
            // after a slow frame or a pause.
            nextJointPublishTime =
                now + 1.0 / Math.Max(0.1, 30);//30 is Hz

            ros.Publish(
                pubTopicName,
                UpdateIntJointsStateStd());
        }

        if (poseChannelMode == PoseChannelMode.Publish &&
            now >= nextPosePublishTime)
        {
            nextPosePublishTime =
                now + 1.0 / Math.Max(0.1,30);

            PublishPose();
            PublishOdometry();
        }
    }


    //private void FixedUpdate()
    //{
    //    bool teleportRobot = visualization;

    //    if (teleportRobot)
    //    {
    //        if (!changedImmovable)
    //        {
    //            previousImmovable = articulationbody.immovable;
    //            changedImmovable = true;
    //        }

    //        PoseTeleport();
    //    }
    //    else
    //    {
    //        RestoreImmovable();
    //    }

    //    if (publishJoints)
    //        ros.Publish(pubTopicName, UpdateIntJointsStateStd());

    //    if (poseChannelMode == PoseChannelMode.Publish)
    //        PublishPose();
    //}

    private void RestoreImmovable()
    {
        if (!changedImmovable || articulationbody == null)
            return;

        articulationbody.immovable = previousImmovable;
        changedImmovable = false;
    }

    //private void JointCallbackSmooth(Int8MultiArrayMsg msg)
    //{
    //    if (!RobotMode || msg == null || msg.data == null ||
    //        msg.data.Length < 12)
    //        return;

    //    _ = control.MoveRobotByThetas(msg.data);
    //}

    private void JointCallbackSmooth(Int8MultiArrayMsg msg)
    {
        if (!receiveJoints ||
            msg == null ||
            msg.data == null ||
            msg.data.Length < 12)
            return;

        _ = control.MoveRobotByThetas(msg.data);
    }

    private void PublishPose()
    {
        var msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = "map",
                stamp = UpdateTime()
            },
            pose = new PoseMsg
            {
                position = morphogear.transform.position.To<FLU>(),
                orientation = morphogear.transform.rotation.To<FLU>()
            }
        };

        ros.Publish(poseTopic, msg);
    }



    private void PublishOdometry()
    {
        Vector3Msg localLinearVelocity = new Vector3Msg();
        Vector3Msg localAngularVelocity = new Vector3Msg();

        if (articulationbody != null)
        {
            // У ArticulationBody свойства velocity и angularVelocity 
            // возвращаются относительно ЛОКАЛЬНЫХ координат объекта.
            // Поэтому мы просто конвертируем их в систему координат ROS (FLU).
            localLinearVelocity = articulationbody.linearVelocity.To<FLU>();
            localAngularVelocity = articulationbody.angularVelocity.To<FLU>();
        }

        // Формируем сообщение OdometryMsg
        var msg = new OdometryMsg
        {
            header = new HeaderMsg
            {
                frame_id = "map",
                stamp = UpdateTime()
            },
            child_frame_id = "base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = morphogear.transform.position.To<FLU>(),
                    orientation = morphogear.transform.rotation.To<FLU>()
                },
                covariance = new double[36] // Фиксированный размер матрицы ковариации
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = localLinearVelocity,
                    angular = localAngularVelocity
                },
                covariance = new double[36] // Фиксированный размер матрицы ковариации
            }
        };

        ros.Publish(odomTopic, msg);
    }

    private void PoseCallback(PoseStampedMsg msg)
    {
        if (poseChannelMode != PoseChannelMode.Receive ||
            msg == null || msg.pose == null)
            return;

        // ROS FLU -> Unity coordinates. Cache the result; TeleportRoot
        // is called from FixedUpdate, not from the ROS callback.
        Vector3 position = msg.pose.position.From<FLU>();
        Quaternion rotation = msg.pose.orientation.From<FLU>();

        lock (poseLock)
        {
            receivedPosition = position;
            receivedRotation = rotation;
            hasReceivedPose = true;
        }
    }

    private void PoseTeleport()
    {
        Vector3 position;
        Quaternion rotation;

        if (teleportPoseSource == TeleportPoseSource.PoseTransform)
        {
            if (poseTransform == null)
                return;

            position = poseTransform.position;
            rotation = poseTransform.rotation;
        }
        else
        {
            // Received poses are available only when the pose channel
            // is explicitly set to Receive.
            if (poseChannelMode != PoseChannelMode.Receive)
                return;

            lock (poseLock)
            {
                if (!hasReceivedPose)
                    return;

                position = receivedPosition;
                rotation = receivedRotation;
            }
        }

        articulationbody.immovable = true;
        articulationbody.TeleportRoot(position, rotation);
    }

    private Int8MultiArrayMsg UpdateIntJointsStateStd()
    {
        articulationbody.GetJointPositions(positions);
        int dofCount = articulationbody.immovable ? 0 : 6;

        sbyte lf_shoulder = ToSByteDegrees(positions[0 + dofCount]);
        sbyte rf_shoulder = ToSByteDegrees(positions[1 + dofCount]);
        sbyte lh_shoulder = ToSByteDegrees(positions[2 + dofCount]);
        sbyte rh_shoulder = ToSByteDegrees(positions[3 + dofCount]);

        sbyte lf_upperarm = ToSByteDegrees(positions[4 + dofCount]);
        sbyte rf_upperarm = ToSByteDegrees(positions[5 + dofCount]);
        sbyte lh_upperarm = ToSByteDegrees(positions[6 + dofCount]);
        sbyte rh_upperarm = ToSByteDegrees(positions[7 + dofCount]);

        sbyte lf_forearm = ToSByteDegrees(positions[8 + dofCount]);
        sbyte rf_forearm = ToSByteDegrees(positions[9 + dofCount]);
        sbyte lh_forearm = ToSByteDegrees(positions[10 + dofCount]);
        sbyte rh_forearm = ToSByteDegrees(positions[11 + dofCount]);

        // Keep the original robot joint ordering.
        sbyte[] data =
        {
            rf_shoulder, rh_shoulder, lh_shoulder, lf_shoulder,
            rf_upperarm, rh_upperarm, lh_upperarm, lf_upperarm,
            rf_forearm, rh_forearm, lh_forearm, lf_forearm
        };

        return new Int8MultiArrayMsg
        {
            data = data,
            layout = new MultiArrayLayoutMsg
            {
                data_offset = 0,
                dim = new[]
                {
                    new MultiArrayDimensionMsg
                    {
                        label = "joints",
                        size = (uint)data.Length,
                        stride = (uint)data.Length
                    }
                }
            }
        };
    }

    private static sbyte ToSByteDegrees(float radians)
    {
        int degrees = Mathf.RoundToInt(radians * Mathf.Rad2Deg);
        return (sbyte)Mathf.Clamp(degrees, sbyte.MinValue, sbyte.MaxValue);
    }


    [Header("Depth publishing")]
    [Min(0.1f)]
    public float depthPublishHz = 30f;
    private float nextDepthPublishTime;

    private void OnDepthUpdated()
    {
        if (!publishDepth || ros == null || rgbdCamera == null)
            return;

        float now = Time.realtimeSinceStartup;

        if (now < nextDepthPublishTime)
            return;

        NativeArray<uint> depth = rgbdCamera.Depth16Data;
        Vector2Int resolution = rgbdCamera.Resolution;

        int pixelCount = resolution.x * resolution.y;

        if (!depth.IsCreated || depth.Length != pixelCount)
            return;

        // Advance the deadline before packing/publishing. Sensor frames
        // arriving before the next deadline are simply discarded.
        nextDepthPublishTime =
            now + 1f / Mathf.Max(0.1f, depthPublishHz);

        // Do not reuse a previous message's data array: the ROS connector
        // may not have finished serializing that message yet.
        byte[] frame = new byte[pixelCount * sizeof(ushort)];
        PackDepthDataUltraFast(depth, frame);

        var depthMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                frame_id = "camera_depth_link",
                stamp = UpdateTime()
            },
            height = (uint)resolution.y,
            width = (uint)resolution.x,
            encoding = "16UC1",
            is_bigendian = 0,
            step = (uint)(resolution.x * sizeof(ushort)),
            data = frame
        };

        ros.Publish(depthTopic, depthMsg);
    }

    //private async void OnDepthUpdated()
    //{
    //    if (!publishDepth) return;
    //    if (isPublishingDepth || !rgbdCamera.Depth16Data.IsCreated) return;

    //    isPublishingDepth = true;

    //    try
    //    {
    //        await Task.Run(() =>
    //            PackDepthDataUltraFast(
    //                rgbdCamera.Depth16Data,
    //                _depth16UC1));

    //        ImageMsg depthMsg = new ImageMsg
    //        {
    //            header = new HeaderMsg
    //            {
    //                frame_id = "camera_depth_link",
    //                stamp = GetCurrentRosTime()
    //            },
    //            height = 480,
    //            width = 640,
    //            encoding = "16UC1",
    //            is_bigendian = 0,
    //            step = 640 * 2,
    //            data = _depth16UC1
    //        };

    //        ros.Publish(depthTopic, depthMsg);
    //    }
    //    finally
    //    {
    //        isPublishingDepth = false;
    //    }
    //}

    private void PackDepthDataUltraFast(
        NativeArray<uint> gpuDepthData,
        byte[] destinationByteArray)
    {
        int length = gpuDepthData.Length;

        unsafe
        {
            fixed (byte* destPtr = destinationByteArray)
            {
                ushort* dest16 = (ushort*)destPtr;

                for (int i = 0; i < length; i++)
                    dest16[i] = (ushort)gpuDepthData[i];
            }
        }
    }

    private TimeMsg GetCurrentRosTime()
    {
        int sec = (int)Time.realtimeSinceStartup;
        uint nanosec =
            (uint)((Time.realtimeSinceStartup - sec) * 1e9);

        return new TimeMsg(sec, nanosec);
    }

    //private TimeMsg UpdateTime()
    //{
    //    var timeSinceEpoch =
    //        DateTime.UtcNow -
    //        new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    //    int sec = (int)timeSinceEpoch.TotalSeconds;
    //    uint nanosec =
    //        (uint)((timeSinceEpoch.TotalSeconds - sec) * 1e9);

    //    return new TimeMsg(sec, nanosec);
    //}

    //private static TimeMsg UpdateTime()
    //{
    //    // Take ONE clock reading for both sec and nanosec.
    //    long unixTicks =
    //        DateTime.UtcNow.Ticks -
    //        new DateTime(
    //            1970, 1, 1, 0, 0, 0,
    //            DateTimeKind.Utc).Ticks;

    //    const long ticksPerSecond = 10_000_000;

    //    int seconds = checked(
    //        (int)(unixTicks / ticksPerSecond));

    //    uint nanoseconds =
    //        (uint)((unixTicks % ticksPerSecond) * 100);

    //    return new TimeMsg(seconds, nanoseconds);
    //}


    private static TimeMsg UpdateTime()
    {
        // 1. Получаем текущее время высокой точности
        DateTimeOffset now = DateTimeOffset.UtcNow;

        // 2. Безопасно приводим секунды к типу int (ROS ожидает int32)
        int seconds = (int)now.ToUnixTimeSeconds();

        // 3. Вычисляем наносекунды из оставшихся миллисекунд
        uint nanoseconds = (uint)(now.Millisecond * 1_000_000);

        return new TimeMsg(seconds, nanoseconds);
    }

}