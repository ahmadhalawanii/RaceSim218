using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class CarDriverAgent : Agent
{
    [Header("Car Components")]
    public CarDriver carDriver;
    public Rigidbody rb;

    [Header("Checkpoint Manager")]
    public checkpoints checkpoints;

    [Header("Ray Perception")]
    public RayPerceptionSensorComponent3D raySensor;

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

        // Subscribe to checkpoint events
        checkpoints.OnCarCorrectCheckpoint += Checkpoints_OnCarCorrectCheckpoint;
        checkpoints.OnCarWrongCheckpoint += Checkpoints_OnCarWrongCheckpoint;
    }

    public override void OnEpisodeBegin()
    {
        // Stop the car completely
        carDriver.StopCompletely();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Reset position and rotation
        transform.position = startPos;
        transform.rotation = startRot;

        // Reset checkpoint distance
        Transform nextCp = checkpoints.GetNextCheckpoint(transform);
        if (nextCp != null)
            lastDistanceToNextCheckpoint = Vector3.Distance(transform.position, nextCp.position);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Ray sensor automatically observes walls/obstacles
        // Add relative direction to next checkpoint
        Transform nextCp = checkpoints.GetNextCheckpoint(transform);
        if (nextCp == null) return;

        Vector3 dirToCheckpoint = (nextCp.position - transform.position).normalized;
        sensor.AddObservation(transform.InverseTransformDirection(dirToCheckpoint));

        // Forward velocity
        float forwardVel = Vector3.Dot(rb.linearVelocity, transform.forward);
        sensor.AddObservation(forwardVel);

        // Distance to next checkpoint
        float dist = Vector3.Distance(transform.position, nextCp.position);
        sensor.AddObservation(dist);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Continuous actions: steer [-1,1], throttle [-1,1]
        float steer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        // Apply inputs
        carDriver.SetInputs(throttle, steer);

        // Penalize backward motion
        float forwardVelocity = Vector3.Dot(rb.linearVelocity, transform.forward);
        if (forwardVelocity < -0.5f) AddReward(-0.01f);

        // Reward approaching next checkpoint
        Transform nextCp = checkpoints.GetNextCheckpoint(transform);
        if (nextCp != null)
        {
            float dist = Vector3.Distance(transform.position, nextCp.position);
            if (dist < lastDistanceToNextCheckpoint)
                AddReward(0.002f);
            else
                AddReward(-0.001f);

            lastDistanceToNextCheckpoint = dist;
        }

        // Small time penalty to encourage faster completion
        AddReward(-0.0002f);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var cont = actionsOut.ContinuousActions;
        cont[0] = Input.GetAxis("Horizontal"); // steer
        cont[1] = Input.GetAxis("Vertical");   // throttle
    }

    private void Checkpoints_OnCarCorrectCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform != transform) return;
        AddReward(1f);
    }

    private void Checkpoints_OnCarWrongCheckpoint(object sender, checkpoints.CarCheckpointEventArgs e)
    {
        if (e.carTransform != transform) return;
        AddReward(-1f);
    }

    private void OnTriggerEnter(Collider other)
    {
        // Only reset if hitting a wall
        if (other.CompareTag("walls"))
        {
            AddReward(-1f);
            ResetCar();
        }

        // Let checkpoints handle themselves
        if (other.CompareTag("checkpoints"))
            checkpoints.CarThroughCheckpoint(other.transform, transform);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("walls"))
        {
            AddReward(-1f);
            ResetCar();
        }
    }

    private void ResetCar()
    {
        carDriver.StopCompletely();
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.position = startPos;
        transform.rotation = startRot;

        Transform nextCp = checkpoints.GetNextCheckpoint(transform);
        if (nextCp != null)
            lastDistanceToNextCheckpoint = Vector3.Distance(transform.position, nextCp.position);
    }
}
