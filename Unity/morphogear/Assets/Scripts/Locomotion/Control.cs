using System;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor;
using UnityEngine;


public enum GaitType
{
    Trot,
    Canter,
    Gallop
}

public class Control : MonoBehaviour
{
    [Header("Configuration")]
    public int rotLook = 30;
    internal int _rotLook = 0; // Made public for the controller
    public float rotStep = 50f;

    internal int initialStandingAngle = 30;

    private int stepAngle = 30;
    private int walkAngle = 0; // Restored to 10 so WalkGait directional checks work

    [Header("Gait Selection")]
    public GaitType selectedGait = GaitType.Trot;



    [Header("Leg References")]
    public GameObject rightForward;
    public GameObject rightHind;
    public GameObject leftForward;
    public GameObject leftHind;

    [Header("Dependencies")]
    [Tooltip("Drag the MorphoGear object here to ensure Calculation is always found.")]
    [SerializeField] private Calculations.Calculation _calculation;

    private Mover[] _rightForward;
    private Mover[] _rightHind;
    private Mover[] _leftForward;
    private Mover[] _leftHind;

    public bool controlRights = true; // Made public for the controller

    internal int L_UA = 155;
    internal int L_FA = 275;
    internal int L_Sh = 95;

    private int speed = 200;

    internal int resetDir = 1;

    // Cancellation source for handling timeouts and preventing infinite deadlocks
    private CancellationTokenSource _activeActionCts;

    void Start()
    {
        if (_calculation == null)
        {
            var gearObj = GameObject.Find("MorphoGear");
            if (gearObj != null) _calculation = gearObj.GetComponent<Calculations.Calculation>();
        }

        _rightForward = rightForward.GetComponentsInChildren<Mover>();
        _rightHind = rightHind.GetComponentsInChildren<Mover>();
        _leftForward = leftForward.GetComponentsInChildren<Mover>();
        _leftHind = leftHind.GetComponentsInChildren<Mover>();

        controlRights = true;
    }

    // =========================================================================
    // Action Wrappers for the External Controller
    // =========================================================================

    public void CancelActiveAction()
    {
        if (_activeActionCts != null) _activeActionCts.Cancel();
    }

    public void ExecuteInitialPosition(int direction, float timeout)
    {
        ExecuteAction(ct => InitialPosition(direction, speed, ct), timeout);
    }

    public void ExecuteStepAction(int direction, float timeout)
    {
        ExecuteAction(ct => ExecuteStep(direction, speed, ct), timeout);
    }

    public void ExecuteRotateBase(float rotation, float timeout)
    {
        ExecuteAction(ct => RotateBase(rotation, speed, ct), timeout);
    }

    public void ExecuteRotatePosition(float angle, float timeout)
    {
        ExecuteAction(ct => RotatePosition(angle, speed, ct), timeout);
    }

    public void ExecuteTeleoperationPosition(int direction, float timeout)
    {
        ExecuteAction(ct => TeleoperationPosition(direction, speed, 0, ct), timeout);
    }


    public void ExecuteSitDown(float timeout)
    {
        ExecuteAction(ct => SitDown(speed, ct), timeout);
    }



    // =========================================================================
    // Timeout & Cancellation Architecture
    // =========================================================================

    private async void ExecuteAction(Func<CancellationToken, Awaitable> action, float timeoutSeconds)
    {
        if (!controlRights) return;
        controlRights = false;

        if (_activeActionCts != null)
        {
            _activeActionCts.Cancel();
            _activeActionCts.Dispose();
        }

        _activeActionCts = new CancellationTokenSource();

        // Replaced .CancelAfter() with a strict Main-Thread timeout runner
        _ = SafeCancelAfterDelay(_activeActionCts, timeoutSeconds);

        try
        {
            await action(_activeActionCts.Token);
        }
        catch (OperationCanceledException)
        {
            Debug.LogWarning($"Action timed out after {timeoutSeconds}s or was manually cancelled.");
        }
        catch (Exception ex)
        {
            Debug.LogError($"Action failed with error: {ex.Message}");
        }
        finally
        {
            controlRights = true;
        }
    }

    // Safely handles timeout cancellation strictly on the Unity Main Thread
    private async Awaitable SafeCancelAfterDelay(CancellationTokenSource cts, float timeoutSeconds)
    {
        try
        {
            await Awaitable.WaitForSecondsAsync(timeoutSeconds, cts.Token);

            // If the time passed and it hasn't been cancelled by a new action, cancel it here
            if (!cts.IsCancellationRequested)
            {
                cts.Cancel();
            }
        }
        catch (OperationCanceledException)
        {
            // This is completely normal: it means a new action was triggered before this one timed out.
        }
    }

    // =========================================================================
    // Helper Methods
    // =========================================================================

    private async Awaitable RotateLegAsync(Mover[] leg, float yaw, float shoulder, float elbow, float spd, CancellationToken ct)
    {
        var t1 = leg[0].RotateTarget(yaw, spd, ct);
        var t2 = leg[1].RotateTarget(shoulder, spd, ct);
        var t3 = leg[2].RotateTarget(elbow, spd, ct);
        await t1; await t2; await t3;
    }

    private void RotateLeg(Mover[] leg, float yaw, float shoulder, float elbow, float spd, CancellationToken ct)
    {
        _ = leg[0].RotateTarget(yaw, spd, ct);
        _ = leg[1].RotateTarget(shoulder, spd, ct);
        _ = leg[2].RotateTarget(elbow, spd, ct);
    }

    // =========================================================================
    // Directional Gait Controller
    // =========================================================================

    private async Awaitable ExecuteStep(int direction, float speedRotation, CancellationToken ct)
    {
        Mover[][] movers = { _rightForward, _rightHind, _leftHind, _leftForward };
        int dirIndex = direction - 1;

        // Directional logical shifting
        Mover[] frontRight = movers[dirIndex];
        Mover[] backRight = movers[(dirIndex + 1) % 4];
        Mover[] backLeft = movers[(dirIndex + 2) % 4];
        Mover[] frontLeft = movers[(dirIndex + 3) % 4];


        switch (selectedGait)
        {
            case GaitType.Trot:
                if ((Mathf.Abs(frontRight[0].articulation.xDrive.target - walkAngle) > 3f) || (Mathf.Abs(backLeft[0].articulation.xDrive.target - walkAngle) > 3f) ||
                    (Mathf.Abs(backRight[1].articulation.xDrive.target + initialStandingAngle) > 3f) || (Mathf.Abs(frontLeft[1].articulation.xDrive.target + initialStandingAngle) > 3f) ||
                    (Mathf.Abs(backRight[2].articulation.xDrive.target + 90 - initialStandingAngle) > 3f) || (Mathf.Abs(frontLeft[2].articulation.xDrive.target + 90 - initialStandingAngle) > 3f) ||
                    (Mathf.Abs(frontRight[1].articulation.xDrive.target + initialStandingAngle) > 3f) || (Mathf.Abs(backLeft[1].articulation.xDrive.target + initialStandingAngle) > 3f) ||
                    (Mathf.Abs(frontRight[2].articulation.xDrive.target + 90 - initialStandingAngle) > 3f) || (Mathf.Abs(backLeft[2].articulation.xDrive.target + 90 - initialStandingAngle) > 3f))
                {
                    await InitialPosition(resetDir, speedRotation, ct);
                }
                resetDir = direction;
                await TrotGait(frontRight, backRight, backLeft, frontLeft, speedRotation / 2, ct);
                break;

            case GaitType.Canter:
                // Auto-align: If we changed direction, legs aren't at WalkAngle. Smoothly reset them.
                if ((Mathf.Abs(frontRight[0].articulation.xDrive.target - walkAngle) > 3f) || (Mathf.Abs(backRight[0].articulation.xDrive.target - walkAngle) > 3f) ||
                  (Mathf.Abs(backLeft[0].articulation.xDrive.target - walkAngle) > 3f) || (Mathf.Abs(frontLeft[0].articulation.xDrive.target - walkAngle) > 3f))
                {
                    await InitialPosition(resetDir, speedRotation, ct);
                }
                resetDir = direction;
                await CanterGait(frontRight, backRight, backLeft, frontLeft, speedRotation, ct);
                break;

            case GaitType.Gallop:
                // Auto-align: If we changed direction, legs aren't at Initial (0) angle. Smoothly reset them.
                if ((Mathf.Abs(frontRight[0].articulation.xDrive.target - walkAngle) > 3f) ||
                    (Mathf.Abs(backLeft[0].articulation.xDrive.target - walkAngle) > 3f))
                {
                    await InitialPosition(resetDir, speedRotation, ct);
                }
                resetDir = direction;
                await GallopGait(frontRight, backRight, backLeft, frontLeft, speedRotation, ct);
                break;
        }

    }

    // =========================================================================
    // Gaits (Clean, readable loops exactly 1 cycle long)
    // =========================================================================

    private async Awaitable TrotGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
    {
        if (Mathf.Abs(bl[0].CurrentPrimaryAxisRotation()) > 0.5f)
            await RotateLegAsync(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(fr[0].CurrentPrimaryAxisRotation()) > 0.5f)
            await RotateLegAsync(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        _ = Halfstep(br, true, speedRotation, 0, ct);
        await Halfstep(fl, false, speedRotation, 0, ct);
        await Stepper(br, fl, bl, fr, speedRotation, ct);
    }

    private async Awaitable CanterGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotat, CancellationToken ct)
    {
        if (_calculation == null) throw new NullReferenceException("Calculation module is missing!");


        int trajectoryHalfPointNumber = 50;
        int totalPoints = trajectoryHalfPointNumber * 2; // 100 iterations exactly per step

        float[][] trajectory = _calculation.trajectoryGenerator(trajectoryHalfPointNumber, 0.9f, 0.5f);
        float[][] angles = _calculation.inverseKinematics(trajectory[0], trajectory[1]);

        // Start from quarter of the phase cycle
        int startIndex = trajectoryHalfPointNumber / 2;
        var speedRotation = 50f;
        // Ensure legs are at the exact starting point of the trajectory cycle concurrently
        var p1 = fr[1].RotateTarget(angles[0][startIndex], speedRotation, ct);
        var p2 = fr[2].RotateTarget(angles[1][startIndex], speedRotation, ct);
        var p3 = fl[1].RotateTarget(angles[0][(startIndex + trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
        var p4 = fl[2].RotateTarget(angles[1][(startIndex + trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
        var p5 = br[1].RotateTarget(angles[0][totalPoints - 1 - startIndex], speedRotation, ct);
        var p6 = br[2].RotateTarget(angles[1][totalPoints - 1 - startIndex], speedRotation, ct);
        var p7 = bl[1].RotateTarget(angles[0][(3 * trajectoryHalfPointNumber - startIndex) % totalPoints], speedRotation, ct);
        var p8 = bl[2].RotateTarget(angles[1][(3 * trajectoryHalfPointNumber - startIndex) % totalPoints], speedRotation, ct);

        await p1; await p2; await p3; await p4; await p5; await p6; await p7; await p8;

        speedRotation = speedRotat * 5;
        // Loop exactly one full cycle (100 ticks)
        for (int step = 0; step < totalPoints && !ct.IsCancellationRequested; step++)
        {
            // Calculate current index wrapped around the array size
            int i = (startIndex + step) % totalPoints;

            var t1 = fr[1].RotateTarget(angles[0][i], speedRotation, ct);
            var t2 = fr[2].RotateTarget(angles[1][i], speedRotation, ct);
            var t3 = fl[1].RotateTarget(angles[0][(i + trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
            var t4 = fl[2].RotateTarget(angles[1][(i + trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
            var t5 = br[1].RotateTarget(angles[0][totalPoints - 1 - i], speedRotation, ct);
            var t6 = br[2].RotateTarget(angles[1][totalPoints - 1 - i], speedRotation, ct);
            var t7 = bl[1].RotateTarget(angles[0][(3 * trajectoryHalfPointNumber - i) % totalPoints], speedRotation, ct);
            var t8 = bl[2].RotateTarget(angles[1][(3 * trajectoryHalfPointNumber - i) % totalPoints], speedRotation, ct);

            await t1; await t2; await t3; await t4; await t5; await t6; await t7; await t8;
            await Awaitable.WaitForSecondsAsync(0.005f, ct);
            await Awaitable.FixedUpdateAsync(ct);
        }
    }


    private async Awaitable GallopGait(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
    {
        if (_calculation == null) throw new NullReferenceException("Calculation module is missing!");

        // Original offset swap you had inside MixWalk
        (fr, br, bl, fl) = (bl, fl, fr, br);

        int trajectoryHalfPointNumber = 50;
        int totalPoints = trajectoryHalfPointNumber * 2; // 100 iterations exactly per step

        float[][] trajectory = _calculation.trajectoryGenerator(trajectoryHalfPointNumber, 0.8f, 0.7f);
        float[][] angles = _calculation.inverseKinematics(trajectory[0], trajectory[1]);

        float[][] trajectory1 = _calculation.trajectoryGenerator(trajectoryHalfPointNumber, 0.8f, 0.7f);
        float[][] angles1 = _calculation.inverseKinematics(trajectory1[0], trajectory1[1]);

        float targetAngle = 30f;
        float rotationChange = 2 * targetAngle / trajectoryHalfPointNumber;
        float rotation = 0.0f;
        float l_max = L_UA * Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad) + L_Sh;
        var startAngle = 0f;


        var t = bl[1].RotateTarget(angles1[0][0], speedRotation, ct);
        var t1 = bl[2].RotateTarget(angles1[1][0], speedRotation, ct);
        var t2 = fr[1].RotateTarget(angles[0][(3 * trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
        var t3 = fr[2].RotateTarget(angles[1][(3 * trajectoryHalfPointNumber) % totalPoints], speedRotation, ct);
        var t4 = br[1].RotateTarget(-initialStandingAngle, speedRotation, ct);
        var t5 = br[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct);
        var t6 = fl[1].RotateTarget(-initialStandingAngle, speedRotation, ct);
        var t7 = fl[2].RotateTarget(-90 + initialStandingAngle, speedRotation, ct);

        await t;
        await t1;
        await t2;
        await t3;
        await t4;
        await t5;
        await t6;
        await t7;

        await Awaitable.WaitForSecondsAsync(1 / speedRotation, ct);

        // Loop exactly one full cycle (100 ticks)
        for (int i = 0; i < totalPoints && !ct.IsCancellationRequested; i++)
        {

            bl[1].RotateTo(angles1[0][i]);
            bl[2].RotateTo(angles1[1][i]);
            fr[1].RotateTo(angles[0][(3 * trajectoryHalfPointNumber - i) % totalPoints]);
            fr[2].RotateTo(angles[1][(3 * trajectoryHalfPointNumber - i) % totalPoints]);

            if (i == (int)(trajectoryHalfPointNumber / 4) - 1)
                startAngle = fl[0].CurrentPrimaryAxisRotation();

            if (i <= trajectoryHalfPointNumber / 4) // 0 to 12
            {
                var slowTarget0 = Mathf.Lerp(-initialStandingAngle, 0, 4f * i / trajectoryHalfPointNumber);
                var slowTarget1 = Mathf.Lerp(-90 + initialStandingAngle, -90, 4f * i / trajectoryHalfPointNumber);
                rotation = 0.0f;
                br[1].RotateTo(slowTarget0);
                br[2].RotateTo(slowTarget1);
                fl[1].RotateTo(slowTarget0);
                fl[2].RotateTo(slowTarget1);

            }
            else if (i <= trajectoryHalfPointNumber * 3 / 4) // 13 to 37
            {

                var slowTarget2 = Mathf.Lerp(startAngle, targetAngle, 2f * i / trajectoryHalfPointNumber - 0.5f);
                fl[0].RotateTo(slowTarget2);//, speedRotation, ct);
                br[0].RotateTo(-slowTarget2);//, speedRotation, ct);
                //await Awaitable.WaitForSecondsAsync(1 / speedRotation, ct);
            }
            else if (i <= trajectoryHalfPointNumber) // 38 to 49
            {
                var slowTarget3 = Mathf.Lerp(0, -initialStandingAngle, 4f * i / trajectoryHalfPointNumber - 3f);
                var slowTarget4 = Mathf.Lerp(-90, -90 + initialStandingAngle, 4f * i / trajectoryHalfPointNumber - 3f);
                br[1].RotateTo(slowTarget3);
                br[2].RotateTo(slowTarget4);
                fl[1].RotateTo(slowTarget3);
                fl[2].RotateTo(slowTarget4);
            }
            else if (i > trajectoryHalfPointNumber)// 50 to 99
            {
                float rotationGoalleft = br[0].CurrentPrimaryAxisRotation() + rotationChange;
                float rotationGoalright = fl[0].CurrentPrimaryAxisRotation() - rotationChange;
                rotation += -Mathf.Sign(br[0].CurrentPrimaryAxisRotation()) * rotationChange;
                float rotationGoalHeight = Mathf.Acos(2 * (Mathf.Cos(rotation * Mathf.Deg2Rad) - 1) * l_max / L_UA + Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad)) * Mathf.Rad2Deg - initialStandingAngle;


                fl[0].RotateTo(rotationGoalright);
                br[0].RotateTo(rotationGoalleft);
                fl[1].RotateTo(-initialStandingAngle - rotationGoalHeight);
                fl[2].RotateTo(-90 + initialStandingAngle + rotationGoalHeight);
                br[1].RotateTo(-initialStandingAngle - rotationGoalHeight);
                br[2].RotateTo(-90 + initialStandingAngle + rotationGoalHeight);
            }
            await Awaitable.WaitForSecondsAsync(1 / (1.5f * speedRotation), ct);
            await Awaitable.FixedUpdateAsync(ct);
        }
    }


    private async Awaitable Stepper(Mover[] right, Mover[] left, Mover[] back, Mover[] forward, float speedRotation, CancellationToken ct)
    {
        float targetAngle = stepAngle;
        float initialRotationright = right[0].CurrentPrimaryAxisRotation();
        float initialRotationleft = left[0].CurrentPrimaryAxisRotation();
        float l_max = L_UA * Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad) + L_Sh;

        float progress = 0f;
        float totalDuration = (targetAngle * 2) / speedRotation;

        while (progress < totalDuration && !ct.IsCancellationRequested)
        {
            progress += Time.fixedDeltaTime;

            float rotationTotal = (progress / totalDuration) * (2 * targetAngle);
            float rotation = rotationTotal < targetAngle ? rotationTotal : 2 * targetAngle - rotationTotal;
            float rotationGoalHeight = Mathf.Acos(2 * (Mathf.Cos(rotation * Mathf.Deg2Rad) - 1) * l_max / L_UA + Mathf.Cos(initialStandingAngle * Mathf.Deg2Rad)) * Mathf.Rad2Deg - initialStandingAngle;

            float rotationGoalleft = initialRotationleft + rotationTotal;
            float rotationGoalright = initialRotationright - rotationTotal;

            right[0].RotateTo(rotationGoalright);
            left[0].RotateTo(rotationGoalleft);
            right[1].RotateTo(-initialStandingAngle - rotationGoalHeight);
            right[2].RotateTo(-90 + initialStandingAngle + rotationGoalHeight);
            left[1].RotateTo(-initialStandingAngle - rotationGoalHeight);
            left[2].RotateTo(-90 + initialStandingAngle + rotationGoalHeight);

            await Awaitable.FixedUpdateAsync(ct);
        }
    }

    private async Awaitable Halfstep(Mover[] limb, bool side, float speedRotation, float grabAngle, CancellationToken ct)
    {
        _ = limb[1].RotateTarget(0, speedRotation, ct);
        await limb[2].RotateTarget(-90, speedRotation, ct);
        await limb[0].RotateTarget(side ? stepAngle : -stepAngle, speedRotation, ct);
        _ = limb[1].RotateTarget(-initialStandingAngle + grabAngle, speedRotation, ct);
        await limb[2].RotateTarget(-90 + initialStandingAngle - grabAngle, speedRotation, ct);
    }

    // =========================================================================
    // Core Movement & Positioning Handlers
    // =========================================================================

    internal async Awaitable InitialPositionDirectional(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
    {

        speedRotation = 75f;
        if (Mathf.Abs(br[0].CurrentPrimaryAxisRotation()) > 3)
        {
            await RotateLegAsync(br, br[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(br, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(br, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else if ((Mathf.Abs(br[1].CurrentPrimaryAxisRotation() + initialStandingAngle) > 3f) || (Mathf.Abs(br[2].CurrentPrimaryAxisRotation() + 90 - initialStandingAngle) > 3f))
        {
            await RotateLegAsync(br, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(br, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else
            RotateLeg(br, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(fl[0].CurrentPrimaryAxisRotation()) > 3)
        {
            await RotateLegAsync(fl, fl[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(fl, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(fl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else if ((Mathf.Abs(fl[1].CurrentPrimaryAxisRotation() + initialStandingAngle) > 3f) || (Mathf.Abs(fl[2].CurrentPrimaryAxisRotation() + 90 - initialStandingAngle) > 3f))
        {
            await RotateLegAsync(fl, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(fl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else
            RotateLeg(fl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(fr[0].CurrentPrimaryAxisRotation()) > 3)
        {
            await RotateLegAsync(fr, fr[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(fr, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else if ((Mathf.Abs(fr[1].CurrentPrimaryAxisRotation() + initialStandingAngle) > 3f) || (Mathf.Abs(fr[2].CurrentPrimaryAxisRotation() + 90 - initialStandingAngle) > 3f))
        {
            await RotateLegAsync(fr, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else
            RotateLeg(fr, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(bl[0].CurrentPrimaryAxisRotation()) > 3)
        {
            await RotateLegAsync(bl, bl[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
            await RotateLegAsync(bl, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else if ((Mathf.Abs(bl[1].CurrentPrimaryAxisRotation() + initialStandingAngle) > 3f) || (Mathf.Abs(bl[2].CurrentPrimaryAxisRotation() + 90 - initialStandingAngle) > 3f))
        {
            await RotateLegAsync(bl, 0, 0, -90, speedRotation, ct);
            await RotateLegAsync(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        }
        else
            RotateLeg(bl, 0, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        await Awaitable.NextFrameAsync(ct);
    }

    internal async Awaitable InitialPosition(int direction, float speedRotation, CancellationToken ct)
    {
        Mover[][] movers = { _rightForward, _rightHind, _leftHind, _leftForward };
        int dirIndex = direction - 1;

        // Directional logical shifting
        Mover[] frontRight = movers[dirIndex];
        Mover[] backRight = movers[(dirIndex + 1) % 4];
        Mover[] backLeft = movers[(dirIndex + 2) % 4];
        Mover[] frontLeft = movers[(dirIndex + 3) % 4];

        await InitialPositionDirectional(frontRight, backRight, backLeft, frontLeft, speedRotation, ct);
    }

    public async Awaitable RotateBase(float rotation, float speedRotation, CancellationToken ct)
    {
        speedRotation = 50;
        var a = _rightForward[0].articulation.xDrive.target;
        var b = _rightHind[0].articulation.xDrive.target;
        var c = _leftForward[0].articulation.xDrive.target;
        var d = _leftHind[0].articulation.xDrive.target;
        if (a != b || a != c || a != d)
            await InitialPosition(resetDir, speedRotation, ct);

        var rotAngle = Mathf.Clamp(rotation, -30, 30);

        RotateLeg(_rightForward, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        RotateLeg(_rightHind, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        RotateLeg(_leftForward, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        await RotateLegAsync(_leftHind, rotAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        await Awaitable.WaitForSecondsAsync(1.0f, ct);
    }

    public async Awaitable RotatePosition(float angle, float speedRotation, CancellationToken ct)
    {
        speedRotation = 75f;
        angle = Mathf.Clamp(angle, -59, 59);

        if (Mathf.Abs(angle) <= 30)
        {
            await RotateBase(angle, speedRotation, ct);
            await InitialPosition(resetDir, speedRotation, ct);
        }
        else
        {
            await RotateBase(-Mathf.Sign(angle) * 30, speedRotation, ct);
            await Restep(angle % 30, speedRotation, ct);
            await RotateBase(0, speedRotation, ct);
        }
    }

    internal async Awaitable Restep(float angle, float speedRotation, CancellationToken ct)
    {
        (Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl) = (_rightForward, _rightHind, _leftHind, _leftForward);

        await RotateLegAsync(br, br[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(br, angle, 0, -90, speedRotation, ct);
        await RotateLegAsync(br, angle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        await RotateLegAsync(fl, fl[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(fl, angle, 0, -90, speedRotation, ct);
        await RotateLegAsync(fl, angle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        await RotateLegAsync(fr, fr[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(fr, angle, 0, -90, speedRotation, ct);
        await RotateLegAsync(fr, angle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        await RotateLegAsync(bl, bl[0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(bl, angle, 0, -90, speedRotation, ct);
        await RotateLegAsync(bl, angle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);


        await Awaitable.NextFrameAsync(ct);
    }

    private int teleopDirection = 1;
    public async Awaitable TeleoperationPosition(int direction, float speedRotation, float grabAngle, CancellationToken ct)
    {
        speedRotation = 100f;
        Mover[][] movers = { _leftHind, _leftForward, _rightForward, _rightHind };
        await InitialPositionDirectional(_rightForward, _rightHind, _leftHind, _leftForward, speedRotation, ct);

        int forward = direction - 1;
        int right = direction % 4;
        int back = (direction + 1) % 4;
        int left = (direction + 2) % 4;

        teleopDirection = back;

        await RotateLegAsync(movers[right], movers[right][0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(movers[right], -30, 0, -90, speedRotation, ct);
        await RotateLegAsync(movers[right], -30, -initialStandingAngle + grabAngle, -90 + initialStandingAngle - grabAngle, speedRotation, ct);

        await RotateLegAsync(movers[left], movers[left][0].CurrentPrimaryAxisRotation(), 0, -90, speedRotation, ct);
        await RotateLegAsync(movers[left], 30, 0, -90, speedRotation, ct);
        await RotateLegAsync(movers[left], 30, -initialStandingAngle + grabAngle, -90 + initialStandingAngle - grabAngle, speedRotation, ct);


        var tf = RotateLegAsync(movers[forward], 0, 0, -90, speedRotation, ct);
        var tb = RotateLegAsync(movers[back], 0, 0, 0, speedRotation, ct);
        await tf; await tb;
    }


    public void Articulate(int index, bool rot)
    {
        Mover[][] movers = { _leftHind, _leftForward, _rightForward, _rightHind };
        var angle = movers[teleopDirection][index].articulation.xDrive.target;
        int sign = rot ? 1 : -1;

        if (index == 0)
            angle = Mathf.Clamp(angle, -30f, 30f);
        else if (index == 1)
            angle = Mathf.Clamp(angle, -90f, 90f);
        else if (index == 2)
            angle = Mathf.Clamp(angle, -90f, 90f);
        movers[teleopDirection][index].RotateTo(angle + sign * 0.1f);
    }

    private bool grab = true;
    public async Awaitable Grab()
    {
        Debug.Log("Grab");
        GameObject[] limb = { leftHind, leftForward, rightForward, rightHind };
        Transform right = limb[teleopDirection].transform.Find("Shoulder/Upperarm/Forearm/Gripper/PivotTip1");
        Transform left = limb[teleopDirection].transform.Find("Shoulder/Upperarm/Forearm/Gripper/PivotTip2");

        Grasper grasper = limb[teleopDirection].GetComponentInChildren<Grasper>();

        float targetAngle = 45f; // Угол, на который расходятся "клешни"
        float duration = 0.3f;   // Скорость анимации
        float elapsed = 0f;

        // Начальные и конечные вращения
        Quaternion startRotR = right.localRotation;
        Quaternion startRotL = left.localRotation;

        // Если grab = true, идем к углу, если false — в Identity (0,0,0)
        Quaternion endRotR = grab ? Quaternion.Euler(0, targetAngle, 0) : Quaternion.identity;
        Quaternion endRotL = grab ? Quaternion.Euler(0, -targetAngle, 0) : Quaternion.identity;

        while (elapsed < duration)
        {
            elapsed += Time.deltaTime;
            float progress = elapsed / duration;

            // Используем SmoothStep для мягкого начала и конца движения
            float curve = Mathf.SmoothStep(0, 1, progress);

            right.localRotation = Quaternion.Slerp(startRotR, endRotR, curve);
            left.localRotation = Quaternion.Slerp(startRotL, endRotL, curve);

            await Awaitable.EndOfFrameAsync();
        }

        grasper.GrabObject(grab);
        // Фиксируем финальные значения
        right.localRotation = endRotR;
        left.localRotation = endRotL;


        grab = !grab;
    }





    private async Awaitable SitDown(float speedRotation, CancellationToken ct)
    {
        speedRotation = 100f;
        if (_calculation == null) throw new NullReferenceException("Calculation module is missing!");

        int trajectoryPointNumber = 100;
        float[][] trajectory = _calculation.trajectorySitDown(trajectoryPointNumber);
        float[][] angles = _calculation.inverseKinematics(trajectory[0], trajectory[1]);

        for (int i = 10; i < trajectoryPointNumber && !ct.IsCancellationRequested; i++)
        {
            var t1 = _rightForward[1].RotateTarget(angles[0][i], speedRotation, ct);
            var t2 = _rightForward[2].RotateTarget(angles[1][i], speedRotation, ct);
            var t3 = _leftForward[1].RotateTarget(angles[0][i], speedRotation, ct);
            var t4 = _leftForward[2].RotateTarget(angles[1][i], speedRotation, ct);
            var t5 = _rightHind[1].RotateTarget(angles[0][i], speedRotation, ct);
            var t6 = _rightHind[2].RotateTarget(angles[1][i], speedRotation, ct);
            var t7 = _leftHind[1].RotateTarget(angles[0][i], speedRotation, ct);
            var t8 = _leftHind[2].RotateTarget(angles[1][i], speedRotation, ct);

            await t1; await t2; await t3; await t4; await t5; await t6; await t7; await t8;
            await Awaitable.FixedUpdateAsync(ct);
        }
    }


    public async Awaitable InitialWalkPositionDirectional(Mover[] fr, Mover[] br, Mover[] bl, Mover[] fl, float speedRotation, CancellationToken ct)
    {
        if (Mathf.Abs(fr[0].CurrentPrimaryAxisRotation() - walkAngle) > 3)
            await RotateLegAsync(fr, walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(br[0].CurrentPrimaryAxisRotation() + walkAngle) > 3)
            await RotateLegAsync(br, -walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(fl[0].CurrentPrimaryAxisRotation() + walkAngle) > 3)
            await RotateLegAsync(fl, -walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        if (Mathf.Abs(bl[0].CurrentPrimaryAxisRotation() - walkAngle) > 3)
            await RotateLegAsync(bl, walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);

        RotateLeg(fr, walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        RotateLeg(br, -walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        RotateLeg(fl, -walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
        await RotateLegAsync(bl, walkAngle, -initialStandingAngle, -90 + initialStandingAngle, speedRotation, ct);
    }


    private CancellationTokenSource interruptToken;

    public async Awaitable MoveRobotByThetas(sbyte[] Angles, float speedRotation = 100)
    {


        interruptToken = new CancellationTokenSource();

        //Debug.Log(Angles[0]);

        _= _rightForward[0].RotateTarget(Angles[0], speedRotation, interruptToken.Token);
        _= _rightForward[1].RotateTarget(Angles[4], speedRotation, interruptToken.Token); // Start loop
        _= _rightForward[2].RotateTarget(Angles[8], speedRotation, interruptToken.Token);
        _= _rightHind[0].RotateTarget(Angles[1], speedRotation , interruptToken.Token);
        _= _rightHind[1].RotateTarget(Angles[5], speedRotation , interruptToken.Token);
        _= _rightHind[2].RotateTarget(Angles[9], speedRotation , interruptToken.Token);
        _= _leftHind[0].RotateTarget(Angles[2], speedRotation, interruptToken.Token);
        _= _leftHind[1].RotateTarget(Angles[6], speedRotation, interruptToken.Token);
        _= _leftHind[2].RotateTarget(Angles[10], speedRotation, interruptToken.Token);
        _= _leftForward[0].RotateTarget(Angles[3], speedRotation, interruptToken.Token);
        _= _leftForward[1].RotateTarget(Angles[7], speedRotation, interruptToken.Token); //Start from half loop
        _= _leftForward[2].RotateTarget(Angles[11], speedRotation, interruptToken.Token);
        await Awaitable.FixedUpdateAsync(interruptToken.Token);

    }





}