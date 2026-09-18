using AgvPlanning;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ROSkmpc : MonoBehaviour
{

    public string TriggerTopicName = "/request_angles";
    public string ExecutePath = "/corner_points";
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private ROSConnection _ros;

    public AgvPathAgent pathAgent;
    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();

        _ros.RegisterPublisher<EmptyMsg>(TriggerTopicName);
        _ros.RegisterPublisher<PoseArrayMsg>(ExecutePath);


    }

    // Update is called once per frame
    void Update()
    {
        
    }


    [ContextMenu("Request KMPC")]
    public void RequestKMPC()
    {
        var pathReq = pathAgent.ActivePath.ToList();
        //Debug.Log("Requested Path Mission " + pathReq.ToString());
        PoseArrayMsg msg = parsePathMission(pathReq);
        _ros.Publish(ExecutePath, msg);

        var res = "Requested Path: ";

        foreach (var path in pathReq)
        {
            res += path.ToString() + "+";
        }

        Debug.Log(res);

        var trigger = new EmptyMsg();
        _ros.Publish(TriggerTopicName, trigger);


    }



    private PoseArrayMsg parsePathMission(List<Vector3> pathReq)
    {
        PoseArrayMsg msg = new PoseArrayMsg();

        List<PoseMsg> pts = new List<PoseMsg>();

        foreach (var pose in pathReq)
        {
            pts.Add(UpdatePoints(pose));
        }

        msg.poses = pts.ToArray();
        return msg;
    }

    private PoseMsg UpdatePoints(Vector3 pose)
    {
        Vector3<FLU> rosPose = pose.To<FLU>();
        Quaternion<FLU> rosQuaternion = Quaternion.identity.To<FLU>();
        PoseMsg poseMsg = new PoseMsg(rosPose, rosQuaternion);
        return poseMsg;
    }
}
