using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class CarDriverAgent : Agent
{
    [SerializeField] private checkpoints checkpoints;
    [SerializeField] private Transform spawnPosition;      // spawn point for wall reset
    [SerializeField] private float speedMultiplier = 5f;  // forward speed
    [SerializeField] private float rayLength = 5f;        // ray distance for wall detection
    [SerializeField] private float rayPenalty = -0.05f;   // penalty for approaching walls

    private CarDriver carDriver;
    private Vector3 lastPosition;

    private void Awake()
    {
        carDriver = GetComponent<CarDriver>();

        // Rigidbody settings
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.isKinematic = false;
            rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
        }
    }

    private void Start()
    {
        checkpoints.OnCarCorrectCheckpoint += checkpoints_OnCarCorrectCheckpoint;
        checkpoints.OnCarWrongCheckpoint += checkpoints_OnCarWrongCheckpoint;
    }

    private void checkpoints_OnCarWrongCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform == transform)
        {
            AddReward(-0.5f); // penalize, no reset
        }
    }

    private void checkpoints_OnCarCorrectCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform == transform)
        {
            AddReward(1f); // reward for correct checkpoint
        }
    }

    public override void OnEpisodeBegin()
    {
        // Only reset internal state, not car position
        carDriver.StopCompletely();
        checkpoints.ResetCheckpoint(transform);
        lastPosition = transform.position;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 checkpointDir = (checkpoints.GetNextCheckpoint(transform).position - transform.position).normalized;
        float directionDot = Vector3.Dot(transform.forward, checkpointDir);
        sensor.AddObservation(directionDot);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Decode actions
        float forwardAmount = 0f;
        float turnAmount = 0f;

        switch (actions.DiscreteActions[0])
        {
            case 0: forwardAmount = 0f; break;
            case 1: forwardAmount = 1f * speedMultiplier; break;
            case 2: forwardAmount = 0f; break; // no reverse
        }

        switch (actions.DiscreteActions[1])
        {
            case 0: turnAmount = 0f; break;
            case 1: turnAmount = +1f; break;
            case 2: turnAmount = -1f; break;
        }

        carDriver.SetInputs(forwardAmount, turnAmount);

        // Reward shaping: distance traveled
        float distanceTravelled = Vector3.Distance(transform.position, lastPosition);
        AddReward(0.1f * distanceTravelled);

        // Reward shaping: direction
        Vector3 checkpointDir = (checkpoints.GetNextCheckpoint(transform).position - transform.position).normalized;
        float directionDot = Vector3.Dot(transform.forward, checkpointDir);
        AddReward(0.05f * directionDot);

        lastPosition = transform.position;

        // Ray-based wall avoidance penalty
        Vector3[] rayDirections = {
            transform.forward,
            Quaternion.Euler(0, 30, 0) * transform.forward,
            Quaternion.Euler(0, -30, 0) * transform.forward
        };

        foreach (Vector3 dir in rayDirections)
        {
            if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength))
            {
                if (hit.collider.CompareTag("walls"))
                {
                    AddReward(rayPenalty);
                }
            }
            Debug.DrawRay(transform.position, dir * rayLength, Color.red);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        int forwardAction = 0;
        if (Input.GetKey(KeyCode.W)) forwardAction = 1;

        int turnAction = 0;
        if (Input.GetKey(KeyCode.D)) turnAction = 1;
        if (Input.GetKey(KeyCode.A)) turnAction = 2;

        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
        discreteActions[0] = forwardAction;
        discreteActions[1] = turnAction;
    }

    // Trigger-based wall reset
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("walls"))
        {
            Debug.Log("Wall hit! Resetting to spawn.");
            AddReward(-1f); // collision penalty
            transform.position = spawnPosition.position + new Vector3(
                Random.Range(-1f, 1f),
                0,
                Random.Range(-1f, 1f)
            );
            transform.rotation = spawnPosition.rotation;
            carDriver.StopCompletely();
            checkpoints.ResetCheckpoint(transform);
            lastPosition = transform.position;
        }

        // Existing checkpoint handling
        if (other.CompareTag("checkpoints"))
        {
            checkpoints.CarThroughCheckpoint(other.transform, transform);
        }
    }
}
