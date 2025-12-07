using UnityEngine;
using System.Collections;

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

    private Rigidbody rb;
    private float stuckTimer = 0f;

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    public void Evaluate(float nearestObstacleDistance, float nearestLocalX)
    {
        float forwardVel = Vector3.Dot(rb != null ? rb.linearVelocity : Vector3.zero, transform.forward);
        if (Mathf.Abs(forwardVel) < stuckSpeed)
        {
            stuckTimer += Time.deltaTime;
        }
        else
        {
            stuckTimer = 0f;
        }

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

        switch (state)
        {
            case State.Navigate:
                speedMultiplier = 1f;
                steerBoost = 0f;
                break;
            case State.AvoidObstacle:
                speedMultiplier = 0.6f;
                steerBoost = -Mathf.Sign(nearestLocalX) * 0.6f;
                break;
            case State.Recover:
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
