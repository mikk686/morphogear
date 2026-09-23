using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry; // Пространство имен для PoseStampedMsg и HeaderMsg
using RosMessageTypes.Std;      // Пространство имен для TimeMsg
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PoseStampedPublisher : MonoBehaviour
{
    [Header("ROS Configuration")]
    public string rosTopic = "waypoint";
    public string frameId = "map"; // Система координат, относительно которой отправляем позу

    [Header("Target Object")]
    public Transform targetObject; // Объект, чью позу мы хотим отправить

    private ROSConnection ros;
    private double lastPublishTime;
    public float publishFrequency = 10f; // Частота отправки в Гц

    void Start()
    {
        // Инициализируем ROS-соединение
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(rosTopic);

        if (targetObject == null)
        {
            targetObject = this.transform; // Если объект не задан, берем текущий
        }
    }

    void Update()
    {

    }

    [ContextMenu("Send point")]
    private void PublishPoseStamped()
    {
        // 1. Получаем текущую глобальную позицию и поворот объекта в Unity
        Vector3 worldPos = targetObject.position;
        Quaternion worldRot = targetObject.rotation;

        // 2. Формируем временную метку ROS
        uint secs = (uint)Time.time;
        uint nsecs = (uint)((Time.time - secs) * 1e9);

        // 3. Создаем сообщение PoseStamped
        PoseStampedMsg poseStamped = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = GetCurrentRosTime()
            },
            pose = new PoseMsg
            {
                // .To<FLU>() автоматически переводит координаты Unity (RUF) в координаты ROS (FLU)
                position = worldPos.To<FLU>(),
                orientation = worldRot.To<FLU>()
            }
        };

        // 4. Публикуем в ROS
        ros.Publish(rosTopic, poseStamped);
    }

    private TimeMsg GetCurrentRosTime()
    {
        int sec = (int)Time.realtimeSinceStartup;
        uint nanosec = (uint)((Time.realtimeSinceStartup - sec) * 1e9);
        return new TimeMsg(sec, nanosec);
    }
}