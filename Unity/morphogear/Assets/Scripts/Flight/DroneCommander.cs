using UnityEngine;

[RequireComponent(typeof(DroneController))]
public class DroneCommander : MonoBehaviour
{
    // Mirrors the capabilities of our QuadrotorController API
    public enum CommanderMode
    {
        PositionControl,
        VelocityControl,
        AttitudeControl,
        RateControl,
        LocalVelocityControl
    }

    private DroneController uav;

    [Header("Active Control Mode")]
    [Tooltip("Select how you want to command the drone. The controller will handle all underlying PIDs.")]
    public CommanderMode currentMode = CommanderMode.PositionControl;

    [Header("Position Mode Settings")]
    [Tooltip("Change this Vector3 in the inspector to move the drone to a specific world coordinate.")]
    public Vector3 targetPosition;

    [Header("Velocity Mode Settings")]
    [Tooltip("Target global velocity (e.g., Z=5 means fly forward at 5m/s).")]
    public Vector3 targetVelocity;

    [Header("Attitude & Rate Settings")]
    [Tooltip("Target Angles (Attitude) or Target Degrees/Sec (Rates)")]
    public Vector3 targetEulerOrRates;
    [Tooltip("Raw thrust force applied when in Attitude or Rate modes (Overrides hover)")]
    public float manualThrust = 10f;

    [Header("Optional Yaw Control (Pos & Vel Modes)")]
    public bool useExplicitYaw = false;
    [Tooltip("If checked, the drone will look at this exact angle. If unchecked, it auto-looks in the direction of flight.")]
    public float explicitYawAngle = 0f;

    void Start()
    {
        uav = GetComponent<DroneController>();

        // Initialize variables to current state so the drone doesn't violently snap to 0,0,0 on start
        targetPosition = transform.position;
        targetVelocity = Vector3.zero;
        targetEulerOrRates = new Vector3(0, transform.eulerAngles.y, 0);
        explicitYawAngle = transform.eulerAngles.y;

        //manualThrust = uav.rb.mass * Mathf.Abs(Physics.gravity.y); // Set to hover thrust
    }

    void FixedUpdate()
    {
        // Nullable float for explicit yaw
        float? yawCommand = useExplicitYaw ? explicitYawAngle : (float?)null;

        // Route the inspector values directly to the QuadrotorController API
        switch (currentMode)
        {
            case CommanderMode.PositionControl:
                // Let the drone's internal Position PIDs handle getting to the target
                uav.CommandPosition(targetPosition, yawCommand);
                break;

            case CommanderMode.VelocityControl:
                // Let the drone's internal Velocity PIDs handle the target speed
                uav.CommandVelocity(targetVelocity, yawCommand);
                break;

            case CommanderMode.AttitudeControl:
                // Force specific Pitch/Yaw/Roll angles and manual thrust
                uav.CommandAttitude(targetEulerOrRates, manualThrust);
                break;

            case CommanderMode.RateControl:
                // Force specific rotational speeds and manual thrust
                uav.CommandRates(targetEulerOrRates, manualThrust);
                break;

            case CommanderMode.LocalVelocityControl:
                Quaternion controlRotation =  Quaternion.Euler(0f, transform.rotation.eulerAngles.y, 0f);
                Vector3 controlVel = controlRotation * targetVelocity;
                uav.CommandVelocity(controlVel, yawCommand);
                break;
        }
    }

    [ContextMenu("Set Target Position to Current")]
    public void SnapTargetToCurrent()
    {
        targetPosition = transform.position;
        targetVelocity = Vector3.zero;
    }
}