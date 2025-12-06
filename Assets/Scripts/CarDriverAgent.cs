using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// CarDriverAgent with 4 control modes: RL, FSM, Fuzzy, Hybrid.
public class CarDriverAgent : Agent
{
    public enum ControlMode { RL, FSM, Fuzzy, Hybrid }

    [Header("Car Components")]
    public CarDriver carDriver;
    public Rigidbody rb;

    [Header("Checkpoint Manager")]
    public checkpoints checkpoints;

    [Header("Ray Perception (for FSM / Fuzzy)")]
    public FuzzyLogic fuzzyLogic;
    public CarFSM fsm;
    public RayPerceptionSensorComponent3D raySensor;

    [Header("Control Mode")]
    public ControlMode controlMode = ControlMode.Hybrid;

    // RL action inputs
    private float rlThrottle = 0f;
    private float rlSteer = 0f;

    // Start/reset
    private Vector3 startPos;
    private Quaternion startRot;
    private float lastDistanceToNextCheckpoint;

    private void Awake()
    {
        if (carDriver == null) carDriver = GetComponent<CarDriver>();
        if (rb == null) rb = GetComponent<Rigidbody>();
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        startPos = transform.position;
        startRot = transform.rotation;

        if (checkpoints != null)
        {
            checkpoints.OnCarCorrectCheckpoint += Checkpoints_OnCarCorrectCheckpoint;
            checkpoints.OnCarWrongCheckpoint += Checkpoints_OnCarWrongCheckpoint;
        }

        if (fuzzyLogic == null) fuzzyLogic = GetComponent<FuzzyLogic>();
        if (fsm == null) fsm = GetComponent<CarFSM>();
    }

    public override void OnEpisodeBegin()
    {
        // Full reset happens here every episode
        carDriver.StopCompletely();

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        transform.position = startPos;
        transform.rotation = startRot;

        if (checkpoints != null)
        {
            checkpoints.ResetCheckpoint(transform);
            Transform next = checkpoints.GetNextCheckpoint(transform);
            if (next != null)
                lastDistanceToNextCheckpoint = Vector3.Distance(transform.position, next.position);
        }
    }

    // Vector obs count = 5  (3 dir + 1 distNorm + 1 velNorm)
    public override void CollectObservations(VectorSensor sensor)
    {
        Transform nextCp = (checkpoints != null) ? checkpoints.GetNextCheckpoint(transform) : null;

        if (nextCp != null)
        {
            Vector3 toCp = (nextCp.position - transform.position).normalized;
            Vector3 localDir = transform.InverseTransformDirection(toCp);
            sensor.AddObservation(localDir); // 3 floats

            float dist = Vector3.Distance(transform.position, nextCp.position);
            sensor.AddObservation(Mathf.Clamp01(dist / 50f)); // 1 float
        }
        else
        {
            sensor.AddObservation(Vector3.zero); // 3
            sensor.AddObservation(1f);           // 1
        }

        float forwardVel = Vector3.Dot(rb.linearVelocity, transform.forward);
        sensor.AddObservation(Mathf.Clamp((forwardVel + 10f) / 40f, 0f, 1f)); // 1 float
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // RL actions
        rlSteer    = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);

        // Allow forward + reverse, but punish reverse hard.
        // If you want no reverse at all, change min to 0f.
        rlThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        // rlThrottle = Mathf.Clamp(actions.ContinuousActions[1], 0f, 1f); // <-- no reverse version

        // Update fuzzy & FSM (used only in non-RL modes)
        if (fuzzyLogic != null) fuzzyLogic.SampleAndCompute();

        float nearestDist = (fuzzyLogic != null && fuzzyLogic.nearestHitDistance < float.MaxValue)
            ? fuzzyLogic.nearestHitDistance
            : Mathf.Infinity;

        float nearestLocalX = 0f;
        if (fuzzyLogic != null && fuzzyLogic.nearestHitDistance < float.MaxValue)
            nearestLocalX = transform.InverseTransformPoint(fuzzyLogic.nearestHitPoint).x;

        if (fsm != null) fsm.Evaluate(nearestDist, nearestLocalX);

        float finalThrottle = 0f;
        float finalSteer = 0f;

        switch (controlMode)
        {
            case ControlMode.RL:
            {
                finalThrottle = rlThrottle;
                finalSteer = rlSteer;
                break;
            }

            case ControlMode.FSM:
            {
                if (fsm != null)
                {
                    finalThrottle = Mathf.Clamp01(fsm.speedMultiplier);
                    finalSteer = Mathf.Clamp(fsm.steerBoost, -1f, 1f);
                }
                break;
            }

            case ControlMode.Fuzzy:
            {
                if (fuzzyLogic != null)
                {
                    finalThrottle = Mathf.Lerp(0f, 1f, fuzzyLogic.fuzzyThrottle);
                    finalSteer = Mathf.Clamp(fuzzyLogic.fuzzySteer, -1f, 1f);
                }
                break;
            }

            case ControlMode.Hybrid:
            default:
            {
                float fuzzyS = (fuzzyLogic != null) ? fuzzyLogic.fuzzySteer : 0f;
                float fsmS   = (fsm != null) ? fsm.steerBoost : 0f;
                finalSteer   = rlSteer + fsmS + fuzzyS * 0.6f;
                finalSteer   = Mathf.Clamp(finalSteer, -1f, 1f);

                float fuzzyThrottle = (fuzzyLogic != null) ? fuzzyLogic.fuzzyThrottle : 0f;
                float fsmThrottle   = (fsm != null) ? fsm.speedMultiplier : 0f;

                finalThrottle = rlThrottle * 0.7f + fuzzyThrottle * 0.2f + fsmThrottle * 0.2f;
                finalThrottle = Mathf.Clamp(finalThrottle, -1f, 1f);

                break;
            }
        }

        carDriver.SetInputs(finalThrottle, finalSteer);

        // ======== REWARD SHAPING ========

        float forwardVelocity = Vector3.Dot(rb.linearVelocity, transform.forward);

        // Strong penalty for going backwards
        if (forwardVelocity < -0.1f)
        {
            AddReward(-0.05f);
        }

        if (checkpoints != null)
        {
            Transform nextCpCheck = checkpoints.GetNextCheckpoint(transform);
            if (nextCpCheck != null)
            {
                float dist = Vector3.Distance(transform.position, nextCpCheck.position);

                if (forwardVelocity > 0f)
                {
                    if (dist < lastDistanceToNextCheckpoint)
                        AddReward(0.02f);  // good progress
                    else
                        AddReward(-0.01f); // drifting away
                }
                else
                {
                    // moving backwards relative to forward
                    AddReward(-0.02f);
                }

                lastDistanceToNextCheckpoint = dist;
            }
        }

        // Small time penalty every step
        AddReward(-0.0002f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Horizontal");
        cont[1] = Input.GetAxis("Vertical");
    }

    private void Checkpoints_OnCarCorrectCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform != transform) return;
        AddReward(1.5f);
    }

    private void Checkpoints_OnCarWrongCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform != transform) return;
        AddReward(-2f);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("walls"))
        {
            AddReward(-2f);   // crash penalty
            ResetCar();       // end episode
        }

        if (other.CompareTag("checkpoints") && checkpoints != null)
            checkpoints.CarThroughCheckpoint(other.transform, transform);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("walls"))
        {
            AddReward(-1f);   // crash penalty
            ResetCar();       // end episode
        }
    }

    /// Ends the current episode (trainer will log stats),
    /// and OnEpisodeBegin() will handle respawn.
    private void ResetCar()
    {
        EndEpisode();
    }
}
