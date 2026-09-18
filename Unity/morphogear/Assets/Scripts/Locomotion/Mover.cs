using System;
using System.Threading;
using UnityEngine;

public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };

public class Mover : MonoBehaviour
{
    public ArticulationBody articulation;

    private void Start()
    {
        if (articulation == null)
            articulation = GetComponent<ArticulationBody>();

        var drive = articulation.xDrive;
        drive.stiffness = 20000f;
        drive.damping = 1000f;
        drive.forceLimit = 1000f;
        articulation.xDrive = drive;
    }

    public async Awaitable RotateTarget(float targetAngle, float speedRotation, CancellationToken ct)
    {
        float startAngle = CurrentPrimaryAxisRotation();//articulation.xDrive.target;
        float deltaAngle = targetAngle - startAngle;

        // Prevent division by zero
        if (Mathf.Abs(speedRotation) < 0.001f) return;

        float duration = Mathf.Abs(deltaAngle) / speedRotation;

        // MODIFICATION 1: If the rotation is so fast that it should finish in less 
        // than one fixed timestep, bypass the loop and snap it instantly.
        if (duration <= Time.fixedDeltaTime)
        {
            RotateTo(Mathf.Clamp(targetAngle, articulation.xDrive.lowerLimit, articulation.xDrive.upperLimit));
            return;
        }

        float elapsed = 0.0f;

        // Bounded loop based on time, not physical position
        while (elapsed < duration)
        {
            if (ct.IsCancellationRequested) break;

            elapsed += Time.fixedDeltaTime;
            float currentTarget = Mathf.Lerp(startAngle, targetAngle, elapsed / duration);

            // Limit boundary check
            if (currentTarget < articulation.xDrive.lowerLimit || currentTarget > articulation.xDrive.upperLimit)
            {
                break;
            }

            RotateTo(currentTarget);

            // Wait for next physics step
            await Awaitable.FixedUpdateAsync(ct);
        }

        // Snap to final target at the end to guarantee precision
        if (!ct.IsCancellationRequested)
        {
            RotateTo(Mathf.Clamp(targetAngle, articulation.xDrive.lowerLimit, articulation.xDrive.upperLimit));
        }
    }
       /*
    // Uses CancellationToken and a deterministic time-based loop
    public async Awaitable RotateTarget(float targetAngle, float speedRotation, CancellationToken ct)
    {
        // Start from the current DRIVE target, NOT the physical rotation.
        // This prevents the math from freezing if the physics body gets stuck.
        float startAngle = articulation.xDrive.target;
        float deltaAngle = targetAngle - startAngle;

        // Prevent division by zero
        if (Mathf.Abs(speedRotation) < 0.001f) return;

        float duration = Mathf.Abs(deltaAngle) / speedRotation;
        float elapsed = 0.0f;

        // Bounded loop based on time, not physical position
        while (elapsed < duration)
        {
            // Exit immediately if the timer runs out or action is cancelled
            if (ct.IsCancellationRequested) break;

            elapsed += Time.fixedDeltaTime;
            float currentTarget = Mathf.Lerp(startAngle, targetAngle, elapsed / duration);

            // Limit boundary check
            if (currentTarget < articulation.xDrive.lowerLimit || currentTarget > articulation.xDrive.upperLimit)
            {
                break;
            }

            RotateTo(currentTarget);

            // Wait for next physics step, passing the cancellation token
            await Awaitable.FixedUpdateAsync(ct);
        }

        // Snap to final target at the end to guarantee precision
        if (!ct.IsCancellationRequested)
        {
            RotateTo(Mathf.Clamp(targetAngle, articulation.xDrive.lowerLimit, articulation.xDrive.upperLimit));
        }
    }    */

    public float CurrentPrimaryAxisRotation()
    {
        float currentRotationRads = articulation.jointPosition[0];
        return Mathf.Rad2Deg * currentRotationRads;
    }

    public void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }
}