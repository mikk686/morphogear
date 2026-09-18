using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(ArticulationBody))]
public class DroneController : MonoBehaviour
{
    // Define the control modes
    public enum FlightMode
    {
        Position,   // Controls X, Y, Z world position
        Velocity,   // Controls X, Y, Z world velocity
        Attitude,   // Controls Pitch, Roll, Yaw angles + manual Thrust
        Rate        // Controls Pitch, Roll, Yaw rotation speeds + manual Thrust
    }

    [System.Serializable]
    public class DroneMotor
    {
        public Transform transform;
        [Tooltip("1 for CCW (generates +Yaw), -1 for CW (generates -Yaw). Alternate these for your Hexrotor.")]
        public float yawDirection = 1f;
    }

    [Header("Flight Mode")]
    public FlightMode currentMode = FlightMode.Position;

    [Header("Propellers Configuration")]
    [Tooltip("Add as many rotors as you need. For Hexrotor, set size to 6.")]
    [SerializeField] private DroneMotor[] motors = new DroneMotor[6];

    [Header("Physics & Hardware")]
    [SerializeField] bool autoCalculateHover = true;
    [SerializeField] float manualHoverForce;
    [SerializeField] float maxPropellerForce = 20f;
    [SerializeField] float yawTorqueMultiplier = 0.5f;
    [SerializeField] float maxSpeed = 25f;

    [Header("PID Gains (Public for Auto-Tuner)")]
    // Position Loop
    public Vector3 PID_pos_x_gains;
    public Vector3 PID_pos_y_gains;
    public Vector3 PID_pos_z_gains;
    // Velocity Loop
    public Vector3 PID_vel_x_gains;
    public Vector3 PID_vel_y_gains;
    public Vector3 PID_vel_z_gains;
    // Attitude (Angle) Loop
    public Vector3 PID_pitch_gains;
    public Vector3 PID_roll_gains;
    public Vector3 PID_yaw_gains;
    // Rate Loop
    public Vector3 PID_pitch_rate_gains;
    public Vector3 PID_roll_rate_gains;
    public Vector3 PID_yaw_rate_gains;

    [Header("Custom Motor Mixer (Auto-Calculated)")]
    [SerializeField] private float[] baseThrustMix;
    [SerializeField] private float[] pitchMix;
    [SerializeField] private float[] rollMix;
    [SerializeField] private float[] yawMix;

    // PID controller instances
    private PIDController PID_pos_x = new PIDController();
    private PIDController PID_pos_y = new PIDController();
    private PIDController PID_pos_z = new PIDController();
    private PIDController PID_vel_x = new PIDController();
    private PIDController PID_vel_y = new PIDController();
    private PIDController PID_vel_z = new PIDController();
    private PIDController PID_pitch = new PIDController();
    private PIDController PID_roll = new PIDController();
    private PIDController PID_yaw = new PIDController();
    private PIDController PID_pitch_rate = new PIDController();
    private PIDController PID_roll_rate = new PIDController();
    private PIDController PID_yaw_rate = new PIDController();

    // Target States
    private Vector3 targetPosition;
    private Vector3 targetVelocity;
    private Vector3 targetAttitude;
    private Vector3 targetRates;
    private float targetThrust;

    // State flags
    private bool useAutoYaw = false;

    // Outputs (Dynamically allocated based on motor count)
    public float[] propForces;
    public ArticulationBody rb { get; private set; }

    private int numMotors => motors.Length;
    private Transform[] blades = new Transform[6];

    void Awake()
    {
        rb = GetComponent<ArticulationBody>();

        BoxCollider boxCol = GetComponent<BoxCollider>();
        if (boxCol != null) rb.centerOfMass = boxCol.center;
        else rb.centerOfMass = Vector3.zero;

        targetPosition = transform.position; // Initialize to current position
        targetAttitude.y = transform.eulerAngles.y;

        // Initialize Dynamic Arrays
        propForces = new float[numMotors];
        baseThrustMix = new float[numMotors];
        pitchMix = new float[numMotors];
        rollMix = new float[numMotors];
        yawMix = new float[numMotors];

        if (numMotors > 0)
        {
            CalculateMotorMixer();
        }
        else
        {
            Debug.LogError("No motors defined in the inspector! Please assign your drone propellers.");
        }

        for (int i = 0; i < numMotors; i++)
        {
            blades[i] = motors[i].transform.Find("13-inch");
        }

        Debug.Log("Mass:" + (rb.mass * Mathf.Abs(Physics.gravity.y)).ToString());
    }

    void FixedUpdate()
    {
        if (numMotors > 0)
        {
            ProcessCascadedControl();
        }
    }

    // ====================================================================
    // ENTRY POINTS FOR EXTERNAL CONTROL (AI, Player Input, Waypoints)
    // ====================================================================

    public void CommandPosition(Vector3 position, float? explicitYaw = null)
    {
        currentMode = FlightMode.Position;
        targetPosition = position;
        HandleYawInput(explicitYaw);
    }

    public void CommandVelocity(Vector3 velocity, float? explicitYaw = null)
    {
        currentMode = FlightMode.Velocity;
        targetVelocity = velocity;
        HandleYawInput(explicitYaw);
    }

    public void CommandAttitude(Vector3 eulerAngles, float thrust)
    {
        currentMode = FlightMode.Attitude;
        targetAttitude = eulerAngles;
        targetThrust = thrust;
        useAutoYaw = false;
    }

    public void CommandRates(Vector3 angularRates, float thrust)
    {
        currentMode = FlightMode.Rate;
        targetRates = angularRates;
        targetThrust = thrust;
        useAutoYaw = false;
    }

    private void HandleYawInput(float? explicitYaw)
    {
        if (explicitYaw.HasValue)
        {
            targetAttitude.y = explicitYaw.Value;
            useAutoYaw = false;
        }
        else
        {
            useAutoYaw = true;
        }
    }

    // ====================================================================
    // CASCADED CONTROL PIPELINE
    // ====================================================================

    private void ProcessCascadedControl()
    {
        float hoverForceTotal = autoCalculateHover ? (rb.mass * Mathf.Abs(Physics.gravity.y)) : manualHoverForce;

        // Dynamically split base thrust across N motors
        float baseThrustPerMotor = hoverForceTotal / numMotors;

        // Auto-Yaw Logic for Position/Velocity modes
        if (useAutoYaw && currentMode <= FlightMode.Velocity)
        {
            Vector3 flightDirection = targetVelocity;
            flightDirection.y = 0f;
            if (flightDirection.sqrMagnitude > 0.01f)
            {
                targetAttitude.y = Mathf.Atan2(flightDirection.x, flightDirection.z) * Mathf.Rad2Deg;
            }
        }

        // 1. POSITION LOOP
        if (currentMode == FlightMode.Position)
        {
            targetVelocity = ApplyPositionControl(targetPosition);
        }

        // 2. VELOCITY LOOP
        if (currentMode <= FlightMode.Velocity)
        {
            ApplyVelocityControl(targetVelocity, out Vector3 desiredAttitude, out float thrustCommand);

            targetAttitude.x = desiredAttitude.x;
            targetAttitude.z = desiredAttitude.z;
            targetThrust = thrustCommand + baseThrustPerMotor;
        }

        // 3. ATTITUDE LOOP
        if (currentMode <= FlightMode.Attitude)
        {
            targetRates = ApplyAttitudeControl(targetAttitude);
        }

        // 4. RATE LOOP
        ApplyAttitudeRateControl(targetRates, targetThrust);
    }

    private Vector3 ApplyPositionControl(Vector3 desiredPos)
    {
        Vector3 posError = desiredPos - transform.position;
        float dt = Time.fixedDeltaTime;

        float velXCmd = PID_pos_x.GetFactorFromPIDController(PID_pos_x_gains, posError.x, dt);
        float velYCmd = PID_pos_y.GetFactorFromPIDController(PID_pos_y_gains, posError.y, dt);
        float velZCmd = PID_pos_z.GetFactorFromPIDController(PID_pos_z_gains, posError.z, dt);

        velXCmd = Mathf.Clamp(velXCmd, -maxSpeed, maxSpeed);
        velYCmd = Mathf.Clamp(velYCmd, -maxSpeed, maxSpeed);
        velZCmd = Mathf.Clamp(velZCmd, -maxSpeed, maxSpeed);

        return new Vector3(velXCmd, velYCmd, velZCmd);
    }

    private void ApplyVelocityControl(Vector3 desiredVel, out Vector3 desiredAttitude, out float throttle)
    {
        Vector3 worldVelError = desiredVel - rb.linearVelocity;

        Quaternion yawRotation = Quaternion.Euler(0f, transform.eulerAngles.y, 0f);
        Vector3 velError = Quaternion.Inverse(yawRotation) * worldVelError;

        float dt = Time.fixedDeltaTime;

        float pitchOut = PID_vel_z.GetFactorFromPIDController(PID_vel_z_gains, velError.z, dt);
        float rollOut = PID_vel_x.GetFactorFromPIDController(PID_vel_x_gains, velError.x, dt);
        throttle = PID_vel_y.GetFactorFromPIDController(PID_vel_y_gains, velError.y, dt);

        pitchOut = Mathf.Clamp(pitchOut, -45f, 45f);
        rollOut = Mathf.Clamp(rollOut, -45f, 45f);

        desiredAttitude = new Vector3(pitchOut, 0f, -rollOut);
    }

    private Vector3 ApplyAttitudeControl(Vector3 desiredAttitude)
    {
        Vector3 currentEuler = transform.rotation.eulerAngles;
        Vector3 err = new Vector3(
            Mathf.DeltaAngle(currentEuler.x, desiredAttitude.x),
            Mathf.DeltaAngle(currentEuler.y, desiredAttitude.y),
            Mathf.DeltaAngle(currentEuler.z, desiredAttitude.z)
        );

        float dt = Time.fixedDeltaTime;
        float pitchRateCmd = PID_pitch.GetFactorFromPIDController(PID_pitch_gains, err.x, dt);
        float yawRateCmd = PID_yaw.GetFactorFromPIDController(PID_yaw_gains, err.y, dt);
        float rollRateCmd = PID_roll.GetFactorFromPIDController(PID_roll_gains, err.z, dt);

        return new Vector3(pitchRateCmd, yawRateCmd, rollRateCmd);
    }

    private void ApplyAttitudeRateControl(Vector3 desiredRates, float thrustPerMotor)
    {
        Vector3 localAngVel = transform.InverseTransformDirection(rb.angularVelocity);
        Vector3 err = desiredRates - localAngVel;

        float dt = Time.fixedDeltaTime;
        float pitchRateOut = PID_pitch_rate.GetFactorFromPIDController(PID_pitch_rate_gains, err.x, dt);
        float rollRateOut = PID_roll_rate.GetFactorFromPIDController(PID_roll_rate_gains, err.z, dt);
        float yawRateOut = PID_yaw_rate.GetFactorFromPIDController(PID_yaw_rate_gains, err.y, dt);

        // Never allow Yaw to steal more than 50% of the motor thrust.
        float maxYawPower = thrustPerMotor * 0.5f;
        yawRateOut = Mathf.Clamp(yawRateOut, -maxYawPower, maxYawPower);

        for (int i = 0; i < numMotors; i++)
        {
            propForces[i] = (thrustPerMotor * baseThrustMix[i])
                          - (pitchRateOut * pitchMix[i])
                          + (rollRateOut * rollMix[i])
                          + (yawRateOut * yawMix[i]);
        }

        ApplyPropForces(propForces);
    }

    private void ApplyPropForces(float[] forces)
    {
        float totalYawTorque = 0f;

        for (int i = 0; i < numMotors; i++)
        {
            float clampedThrust = Mathf.Clamp(forces[i], 0f, maxPropellerForce);
            rb.AddForceAtPosition(transform.up * clampedThrust, motors[i].transform.position, ForceMode.Force);

            // Accumulate yaw torque based on individual rotor spin directions
            totalYawTorque += clampedThrust * yawMix[i] * yawTorqueMultiplier;

            blades[i].Rotate(Vector3.up, (5f+clampedThrust) * motors[i].yawDirection);
        }

        rb.AddTorque(transform.up * totalYawTorque, ForceMode.Force);
    }

    // ====================================================================
    // INITIALIZATION & RESET
    // ====================================================================

    private void CalculateMotorMixer()
    {
        Vector3[] localPos = new Vector3[numMotors];
        float maxX = 0f, maxZ = 0f;

        // Step 1: Find relative positions and maximum spread
        for (int i = 0; i < numMotors; i++)
        {
            localPos[i] = transform.InverseTransformPoint(motors[i].transform.position) - rb.centerOfMass;
            maxX = Mathf.Max(maxX, Mathf.Abs(localPos[i].x));
            maxZ = Mathf.Max(maxZ, Mathf.Abs(localPos[i].z));
        }

        // Step 2: Assign generalized weights based on geometry
        for (int i = 0; i < numMotors; i++)
        {
            // Assuming symmetric geometric design where all motors carry equal weight
            baseThrustMix[i] = 1f;

            // Pitch control effectiveness (Front/Back)
            pitchMix[i] = maxZ > 0.01f ? (localPos[i].z / maxZ) : 0f;

            // Roll control effectiveness (Right/Left)
            rollMix[i] = maxX > 0.01f ? (localPos[i].x / maxX) : 0f;

            // Yaw torque multiplier is given by the user in the Inspector
            yawMix[i] = motors[i].yawDirection;
        }
    }

    public float GetCurrentThrust()
    {
        float thrust = 0f;
        for (int i = 0; i < numMotors; i++)
        {
            thrust += propForces[i];
        }
        return thrust;
    }

    public void ResetDroneState()
    {
        rb.immovable = true;
        transform.rotation = Quaternion.identity;
        rb.immovable = false;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        rb.Sleep();
        rb.WakeUp();

        targetPosition = transform.position;
        targetVelocity = Vector3.zero;
        targetAttitude = Vector3.zero;
        targetRates = Vector3.zero;
        targetThrust = 0f;
        useAutoYaw = false;

        for (int i = 0; i < numMotors; i++)
        {
            propForces[i] = 0f;
        }

        PID_pos_x.Reset(); PID_pos_y.Reset(); PID_pos_z.Reset();
        PID_vel_x.Reset(); PID_vel_y.Reset(); PID_vel_z.Reset();
        PID_pitch.Reset(); PID_roll.Reset(); PID_yaw.Reset();
        PID_pitch_rate.Reset(); PID_roll_rate.Reset(); PID_yaw_rate.Reset();
    }
}

public class PIDController
{
    private float error_old = 0f;
    private float error_sum = 0f;
    public float error_sumMax = 20f;

    public float GetFactorFromPIDController(Vector3 gains, float error, float deltaTime)
    {
        if (deltaTime <= 0f) return 0f;

        float output = 0f;
        output += gains.x * error; // P

        error_sum += error * deltaTime;
        error_sum = Mathf.Clamp(error_sum, -error_sumMax, error_sumMax);
        output += gains.y * error_sum; // I

        float d_dt_error = (error - error_old) / deltaTime;
        error_old = error;
        output += gains.z * d_dt_error; // D

        return output;
    }

    public void Reset() { error_old = 0f; error_sum = 0f; }
}