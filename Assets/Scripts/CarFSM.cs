using UnityEngine;
using System.Collections;

/// Very small Finite State Machine for high-level car behavior.
/// Exposes two outputs:
///   - speedMultiplier (0..1) to scale throttle
///   - steerBoost (-1..1) to bias steering away from obstacles
/// 
/// Conditions are evaluated each Update(); FSM is intentionally simple and deterministic.
public class CarFSM : MonoBehaviour
{
    public enum State
    {
        Idle,
        Navigate,
        AvoidObstacle,
        Recover
    }

    [Header("FSM Settings")]
    public State state = State.Navigate;

    [Tooltip("Distance under which we consider obstacle 'close' and switch to AvoidObstacle")]
    public float avoidDistance = 6f;

    [Tooltip("Distance under which we consider collision and recover")]
    public float collisionDistance = 1.0f;

    [Tooltip("If forward speed < stuckSpeed for stuckTime seconds, go to Recover")]
    public float stuckSpeed = 0.5f;
    public float stuckTime = 1.5f;

    [Header("FSM outputs")]
    [Range(0.1f, 1f)]
    public float speedMultiplier = 1f;
    [Range(-1f, 1f)]
    public float steerBoost = 0f;

    // internals
    private Rigidbody rb;
    private float stuckTimer = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    /// Evaluate the FSM given sensor inputs: nearest obstacle distance and local X of obstacle.
    /// Call this every update/frame.
    public void Evaluate(float nearestObstacleDistance, float nearestLocalX)
    {
        // Update stuck timer
        float forwardVel = Vector3.Dot(rb != null ? rb.linearVelocity : Vector3.zero, transform.forward);
        if (Mathf.Abs(forwardVel) < stuckSpeed)
        {
            stuckTimer += Time.deltaTime;
        }
        else
        {
            stuckTimer = 0f;
        }

        // Determine state transitions
        if (stuckTimer >= stuckTime)
        {
            state = State.Recover;
        }
        else if (nearestObstacleDistance <= collisionDistance)
        {
            state = State.Recover;
        }
        else if (nearestObstacleDistance <= avoidDistance)
        {
            state = State.AvoidObstacle;
        }
        else
        {
            state = State.Navigate;
        }

        // Compute outputs for each state
        switch (state)
        {
            case State.Navigate:
                speedMultiplier = 1f;
                steerBoost = 0f;
                break;
            case State.AvoidObstacle:
                // slow a bit and bias steering away from obstacle (use sign of nearestLocalX)
                speedMultiplier = 0.6f;
                steerBoost = -Mathf.Sign(nearestLocalX) * 0.6f; // push away
                break;
            case State.Recover:
                // stop and orient to spawn or try small reverse to get free
                speedMultiplier = 0.2f;
                steerBoost = 0f;
                break;
            case State.Idle:
            default:
                speedMultiplier = 0f;
                steerBoost = 0f;
                break;
        }
    }
}
