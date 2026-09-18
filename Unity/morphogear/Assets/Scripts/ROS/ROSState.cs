// =============================================================================
//  RobotStateDisplay.cs
//
//  Subscribes to /robot_state (std_msgs/String) from the morphogear ROS node
//  and displays:
//      - Gait:  Trot | Canter | Gallop
//      - Mode:  ManualController | TeleopDirection | TeleopHandControl
//      - Busy:  1 (action in progress) | 0 (idle)
//
//  Requires: Unity ROS-TCP-Connector + TextMeshPro
//
//  Wire format (CSV):  gait,mode,busy
// =============================================================================

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using TMPro;  // TextMeshPro

public class RobotStateDisplay : MonoBehaviour
{
    [Header("ROS")]
    public string stateTopic = "/robot_state";

    [Header("UI Elements (assign in Inspector)")]
    public TextMeshProUGUI gaitText;
    public TextMeshProUGUI modeText;
    public TextMeshProUGUI busyText;

    private ROSConnection _ros;

    public GameObject cameraObject;

    void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<StringMsg>(stateTopic, OnStateReceived);

        // Default text while waiting for first message
        if (gaitText) gaitText.text = "Gait: ---";
        if (modeText) modeText.text = "Mode: ---";
        if (busyText) busyText.text = "Status: ---";
    }

    private void OnStateReceived(StringMsg msg)
    {
        // Parse CSV:  gait,mode,busy
        var parts = msg.data.Split(',');
        if (parts.Length < 3) return;

        string gait = parts[0];
        string mode = parts[1];
        string busy = parts[2];

        // Update UI
        if (gaitText) {

            gaitText.text = $"Gait: {gait}";

            if (gait == "Trot")   {cameraObject.transform.localPosition = new Vector3(0f, 0.3f, -1.2f); cameraObject.transform.localRotation = Quaternion.Euler(10f, 0f, 0f);   }
            if (gait == "Canter") {cameraObject.transform.localPosition = new Vector3(1f, 0.5f, -1f);   cameraObject.transform.localRotation = Quaternion.Euler(15f, -45f, 0f); }
            if (gait == "Gallop") {cameraObject.transform.localPosition = new Vector3(0f, 0.5f, -1.5f); cameraObject.transform.localRotation = Quaternion.Euler(15f, 0f, 0f);   }

    }
        if (modeText) modeText.text = $"Mode: {mode}";
        if (busyText)
        {
            if (busy == "1")
            {
                busyText.text = "Status: <color=red>BUSY</color>";
            }
            else
            {
                busyText.text = "Status: <color=green>IDLE</color>";
            }
        }
    }
}