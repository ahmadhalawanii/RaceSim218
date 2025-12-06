using UnityEngine;

/// Simple fuzzy logic controller that examines rays (matching your RayPerception settings)
/// and produces two outputs:
///   - fuzzyThrottle (0..1) multiplier (1 = full speed, smaller = slow down)
///   - fuzzySteer (-1..1) steer correction to avoid nearest obstacle
///
/// This version adds angle-weighting so walls directly to the side don't cause huge steering
/// away from them (fixing the "side cars dart into the middle car" issue).
public class FuzzyLogic : MonoBehaviour
{
    [Header("Raycast (match the RayPerception settings)")]
    public int raysPerDirection = 6;   // half on each side (6 => total 13)
    public float maxRayDegrees = 70f;
    public float rayLength = 20f;
    public float sphereRadius = 1f;
    public LayerMask rayLayerMask;

    [Header("Fuzzy distance thresholds (meters)")]
    [Tooltip("Within this distance we consider the wall 'near' and react strongly (mostly in front).")]
    public float nearThreshold = 6f;

    [Tooltip("Within this distance we are cautious but not in full panic.")]
    public float mediumThreshold = 12f;

    [Tooltip("Ignore extremely tiny distances to avoid noisy hits.")]
    public float minDistanceToConsider = 0.2f;

    [Header("Outputs scaling")]
    [Range(0.0f, 1.0f)]
    [Tooltip("Minimum throttle multiplier when very close to a wall (0.25 = slow to 25% speed).")]
    public float minThrottleMultiplier = 0.25f;

    [Range(0f, 1f)]
    [Tooltip("Base how strongly steering will attempt to correct. This is scaled by angle too.")]
    public float steerStrength = 0.9f;

    [Header("Angle weighting")]
    [Range(0f, 1f)]
    [Tooltip("How much we care about obstacles mostly to the SIDE. 0.3 = side walls cause only 30% of normal steer.")]
    public float sideSteerWeight = 0.3f;

    // Results (read-only, useful for debugging)
    [HideInInspector] public float fuzzyThrottle = 1f;
    [HideInInspector] public float fuzzySteer = 0f;
    [HideInInspector] public float nearestHitDistance = 999f;
    [HideInInspector] public Vector3 nearestHitPoint = Vector3.zero;
    [HideInInspector] public float nearestForwardFactor = 0f; // 0 = side, 1 = straight ahead

    void Awake()
    {
        fuzzyThrottle = 1f;
        fuzzySteer = 0f;
        nearestHitDistance = float.MaxValue;
        nearestForwardFactor = 0f;
    }

    /// Call this each physics frame (or from Agent) to sample the rays and update fuzzy outputs.
    public void SampleAndCompute()
    {
        int totalRays = raysPerDirection * 2 + 1;
        float minAngle = -maxRayDegrees;
        float maxAngle = maxRayDegrees;

        nearestHitDistance = float.MaxValue;
        nearestHitPoint = Vector3.zero;
        nearestForwardFactor = 0f;

        Vector3 origin = transform.position + Vector3.up * 0.5f; // small vertical offset

        for (int i = 0; i < totalRays; i++)
        {
            float t = (totalRays == 1) ? 0.5f : (float)i / (totalRays - 1);
            float angle = Mathf.Lerp(minAngle, maxAngle, t);

            // local direction relative to the car's forward
            Vector3 localDir = Quaternion.Euler(0f, angle, 0f) * Vector3.forward;
            Vector3 worldDir = transform.TransformDirection(localDir);

            if (Physics.SphereCast(origin, sphereRadius, worldDir, out RaycastHit hit, rayLength, rayLayerMask, QueryTriggerInteraction.Ignore))
            {
                if (hit.distance < nearestHitDistance && hit.distance > minDistanceToConsider)
                {
                    nearestHitDistance = hit.distance;
                    nearestHitPoint = hit.point;

                    // How "in front" is this hit? Dot product between forward and direction to hit.
                    Vector3 toHitDir = (hit.point - origin).normalized;
                    nearestForwardFactor = Mathf.Clamp01(Vector3.Dot(transform.forward, toHitDir));
                    // 1.0  => directly ahead
                    // 0.0  => exactly sideways
                    // (we clamp negative to 0, so things behind the car are treated as "side" at most)
                }
            }
        }

        // No obstacle detected
        if (nearestHitDistance == float.MaxValue)
        {
            fuzzyThrottle = 1f;
            fuzzySteer = 0f;
            return;
        }

        // --- Distance-based fuzzy memberships ---
        float near = MembershipShoulderNear(nearestHitDistance, nearThreshold);
        float medium = MembershipTriangular(
            nearestHitDistance,
            nearThreshold * 0.8f,
            (nearThreshold + mediumThreshold) * 0.5f,
            mediumThreshold * 1.05f
        );

        float far = Mathf.Clamp01((nearestHitDistance - (mediumThreshold * 0.9f)) /
                                  (rayLength - mediumThreshold * 0.9f));

        // --- Throttle control ---
        // Reduce throttle when near/medium are high.
        float throttleReduction = Mathf.Clamp01(near * 1.0f + medium * 0.45f);
        fuzzyThrottle = Mathf.Lerp(1f, minThrottleMultiplier, throttleReduction);

        // --- Steering control (angle-weighted) ---
        // Base steer strength from distance memberships.
        float steerAwayStrength = (near * 1.0f + medium * 0.4f);
        steerAwayStrength = Mathf.Clamp01(steerAwayStrength);

        // Angle weighting:
        //  - nearestForwardFactor = 1 => obstacle straight ahead => full steerStrength
        //  - nearestForwardFactor = 0 => obstacle sideways => sideSteerWeight * steerStrength
        float angleWeight = Mathf.Lerp(sideSteerWeight, 1f, nearestForwardFactor);
        steerAwayStrength *= angleWeight;

        // Decide steer direction: obstacle on right -> steer left (negative), and vice versa.
        Vector3 localHit = transform.InverseTransformPoint(nearestHitPoint);
        float steerDir = Mathf.Sign(localHit.x); // +1 = obstacle to right, -1 = obstacle to left
        fuzzySteer = -steerDir * steerAwayStrength * steerStrength;
    }

    // Simple triangular membership (a,b,c)
    private float MembershipTriangular(float x, float a, float b, float c)
    {
        if (x <= a || x >= c) return 0f;
        if (Mathf.Approximately(x, b)) return 1f;
        if (x > a && x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    // Shoulder membership for "near" (1 at distance<=threshold, then falls)
    private float MembershipShoulderNear(float x, float threshold)
    {
        float a = threshold * 0.6f;
        float c = threshold * 1.6f;

        if (x <= a) return 1f;
        if (x >= c) return 0f;

        // linear falloff between a and c
        return Mathf.Clamp01((c - x) / (c - a));
    }
}
