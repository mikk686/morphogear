using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.InputSystem;

//При переключении на внешнее управление, сделать свободную 3DS камеру
//Сделать респаун
public enum ControlMode
{
    Normal,
    WaitingForTeleopDirection,
    TeleopHandControl,
    Flight
}

[RequireComponent(typeof(Control))]
public class KeyboardController : MonoBehaviour
{
    [Header("Controller Setup")]
    public Control control;
    public DroneCommander commander;
    public Transform cameraObject;

    public ControlMode _currentMode = ControlMode.Normal;
    private ArticulationBody MG;
    private void Awake()
    {
        if (control == null)
        {
            control = GetComponent<Control>();

        }
        MG = GetComponent<ArticulationBody>();
    }

    private Vector3 tpPose = new Vector3(0f,0.6f,0f);
    void Update()
    {
        var kb = Keyboard.current;
        if (kb == null) return;

        if (kb.backquoteKey.wasPressedThisFrame)
        {
            Debug.Log("Toggled Manual Control Rights");
            control.CancelActiveAction();
            control.controlRights = !control.controlRights;
        }

        if (kb.digit4Key.wasPressedThisFrame)
        {
            // Instantly stop whatever the robot is currently doing
            control.CancelActiveAction();

            // Force unlock control rights in case an action was in progress
            control.controlRights = true;

            if (_currentMode == ControlMode.Normal)
            {
                // Switch TO Teleop
                _currentMode = ControlMode.WaitingForTeleopDirection;
                Debug.Log("Entered Teleop Mode. Waiting for direction (W/A/S/D)...");
            }
            else
            {
                cameraObject.transform.localPosition = new Vector3(0f, 0.3f, -1.2f); cameraObject.transform.localRotation = Quaternion.Euler(10f, 0f, 0f);
                // Switch BACK to Normal
                _currentMode = ControlMode.Normal;
                control.ExecuteInitialPosition(control.resetDir, 20f);
                Debug.Log("Exited Teleop Mode. Returning to Initial Position.");
            }
            return; // Skip processing other inputs on the exact frame we switch modes
        }

        if (kb.tKey.wasPressedThisFrame && _currentMode!=ControlMode.Flight)
        {
            _=PrepareAndTakeOff();
        }

        if (kb.gKey.wasPressedThisFrame && _currentMode == ControlMode.Flight)
        {
            _ = PrepareAndLanding();
        }

        if (kb.rKey.wasPressedThisFrame)
        {
            MG.TeleportRoot(tpPose, Quaternion.identity);
        }



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
                ManualFlight(kb);
                break;
        }

    }


    async Awaitable PrepareAndTakeOff()
    {
        cameraObject.transform.localPosition = new Vector3(0f, 1f, -2f); 
        cameraObject.transform.localRotation = Quaternion.Euler(20f, 0f, 0f);
        control.ExecuteInitialPosition(control.resetDir, 20f);
        await Awaitable.WaitForSecondsAsync(5f);
        control.ExecuteSitDown(5f);
        await Awaitable.WaitForSecondsAsync(5f);
        control.controlRights= false;
        commander.currentMode = DroneCommander.CommanderMode.PositionControl;
        commander.targetPosition= transform.position+2*Vector3.up;
        await Awaitable.WaitForSecondsAsync(5f);
        _currentMode = ControlMode.Flight;
        lastPose = commander.targetPosition;
    }

    async Awaitable PrepareAndLanding()
    {
        _currentMode = ControlMode.Normal;
        lastPose = transform.position;    
        commander.currentMode = DroneCommander.CommanderMode.VelocityControl;
        commander.targetVelocity = Vector3.down;
        await Awaitable.WaitForSecondsAsync(5f);
        control.controlRights = true;
        control.ExecuteInitialPosition(control.resetDir, 20f);
        commander.manualThrust = 0;
        commander.targetPosition = Vector3.zero;
        commander.targetVelocity = Vector3.zero;
        commander.targetEulerOrRates = Vector3.zero;
        commander.currentMode = DroneCommander.CommanderMode.AttitudeControl;


    }


    private Vector3 lastPose;
    void ManualFlight(Keyboard kb)
    {
        if (kb.anyKey.wasReleasedThisFrame)
            lastPose = transform.position;

        if (kb.anyKey.isPressed)
        {
            commander.currentMode = DroneCommander.CommanderMode.LocalVelocityControl;
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
            commander.currentMode = DroneCommander.CommanderMode.PositionControl;
        }

        if (kb.eKey.isPressed)
            commander.explicitYawAngle += 1f;
        if (kb.qKey.isPressed)
            commander.explicitYawAngle -= 1f;
        if (kb.xKey.wasPressedThisFrame)
            commander.explicitYawAngle = 0f;




    }

    void ManualController(Keyboard kb)
    {
        //Priority Command Launch
        if (kb.xKey.wasPressedThisFrame) { control.ExecuteInitialPosition(control.resetDir, 20f); }
        else if (kb.wKey.isPressed) { control.ExecuteStepAction(1, 12f); }
        else if (kb.sKey.isPressed) { control.ExecuteStepAction(3, 12f); }
        else if (kb.dKey.isPressed) { control.ExecuteStepAction(2, 12f); }
        else if (kb.aKey.isPressed) { control.ExecuteStepAction(4, 12f); }

        else if (kb.eKey.wasPressedThisFrame) { control.ExecuteRotateBase(control._rotLook, 5f); control._rotLook = (control._rotLook != 0) ? 0 : control.rotLook; }
        else if (kb.qKey.wasPressedThisFrame) { control.ExecuteRotateBase(-control._rotLook, 5f); control._rotLook = (control._rotLook != 0) ? 0 : control.rotLook; }

        else if (kb.cKey.wasPressedThisFrame) { control.ExecuteRotatePosition(-control.rotStep, 20f); }
        else if (kb.zKey.wasPressedThisFrame) { control.ExecuteRotatePosition(control.rotStep, 20f); }

        if (kb.digit1Key.wasPressedThisFrame) { control.selectedGait = GaitType.Trot; cameraObject.transform.localPosition = new Vector3(0f, 0.3f, -1.2f); cameraObject.transform.localRotation = Quaternion.Euler(10f, 0f, 0f); }
        if (kb.digit2Key.wasPressedThisFrame) { control.selectedGait = GaitType.Canter; cameraObject.transform.localPosition = new Vector3(1f, 0.5f, -1f); cameraObject.transform.localRotation = Quaternion.Euler(15f, -45f, 0f); }
        if (kb.digit3Key.wasPressedThisFrame) { control.selectedGait = GaitType.Gallop; cameraObject.transform.localPosition = new Vector3(0f, 0.5f, -1.5f); cameraObject.transform.localRotation = Quaternion.Euler(15f, 0f, 0f); }
    }

    void TeleopDirectionController(Keyboard kb)
    {
        int direction = 0;
        if (kb.wKey.wasPressedThisFrame) direction = 1;
        else if (kb.dKey.wasPressedThisFrame) direction = 2;
        else if (kb.sKey.wasPressedThisFrame) direction = 3;
        else if (kb.aKey.wasPressedThisFrame) direction = 4;

        control.resetDir = (direction + 2) % 4 + 1;

        // Once a direction is pressed, trigger the action and move to Hand Control state
        if (direction != 0)
        {
            Debug.Log($"Direction {direction} chosen. Transitioning to Teleoperation pose.");
            _currentMode = ControlMode.TeleopHandControl;

            // Execute the position action. controlRights will lock automatically until it finishes,
            // preventing the user from accidentally sending hand commands while it's still positioning.
            control.ExecuteTeleoperationPosition(direction, 30f);


            Vector3 baseOffset = new Vector3(0f, 0.7f, 0.2f);
            float angleY = (direction - 1) * 90f;
            Quaternion rotationY = Quaternion.Euler(0, angleY, 0);
            cameraObject.transform.localPosition = rotationY * baseOffset;
            cameraObject.transform.localRotation = rotationY * Quaternion.Euler(90f, 0f, 0f);
        }


    }

    void TeleopHandController(Keyboard kb)
    {
        if (kb.qKey.isPressed) { control.Articulate(0, true);   }
        if (kb.wKey.isPressed) { control.Articulate(0, false); }
        if (kb.aKey.isPressed) { control.Articulate(1, true);  }
        if (kb.zKey.isPressed) { control.Articulate(1, false); }
        if (kb.sKey.isPressed) { control.Articulate(2, true);  }
        if (kb.xKey.isPressed) { control.Articulate(2, false); }
        if (kb.spaceKey.wasPressedThisFrame) { _ = control.Grab(); }
    }
}