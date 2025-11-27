using System.Linq;
using UnityEngine;

/// Simple fuzzy logic controller that examines rays (matching your RayPerception settings)
/// and produces two outputs:
///   - fuzzyThrottle (0..1) multiplier (1 = full speed, smaller = slow down)
///   - fuzzySteer (-1..1) steer correction to avoid nearest obstacle
/// 
/// Designed to be non-invasive and easy to tune from the Inspector.
public class FuzzyLogic : MonoBehaviour
{
    [Header("Raycast (match the RayPerception settings)")]
    public int raysPerDirection = 6;   // half on each side (6 => total 13)
    public float maxRayDegrees = 70f;
    public float rayLength = 20f;
    public float sphereRadius = 1f;
    public LayerMask rayLayerMask;

    [Header("Fuzzy thresholds (meters)")]
    public float nearThreshold = 6f;
    public float mediumThreshold = 12f; // medium spans 5..12 in design
    public float minDistanceToConsider = 0.2f; // avoid div by 0

    [Header("Outputs scaling")]
    [Range(0.0f, 1.0f)]
    public float minThrottleMultiplier = 0.25f; // how slow we allow fuzzy to make us
    [Range(0f, 1f)]
    public float steerStrength = 0.9f; // how strongly fuzzy steering will attempt to correct

    // Results (read-only)
    public float fuzzyThrottle = 1f;
    public float fuzzySteer = 0f;
    public float nearestHitDistance = 999f;
    public Vector3 nearestHitPoint = Vector3.zero;

    void Awake()
    {
        // Ensure sensible defaults
        fuzzyThrottle = 1f;
        fuzzySteer = 0f;
        nearestHitDistance = float.MaxValue;
    }

    /// Call this each physics frame (or from Agent) to sample the rays and update fuzzy outputs.
    public void SampleAndCompute()
    {
        // Cast rays in a horizontal fan
        int totalRays = raysPerDirection * 2 + 1;
        float minAngle = -maxRayDegrees;
        float maxAngle = maxRayDegrees;

        nearestHitDistance = float.MaxValue;
        nearestHitPoint = Vector3.zero;
        float nearestLocalX = 0f; // local X sign to determine left/right

        Vector3 origin = transform.position + Vector3.up * 0.5f; // small vertical offset

        for (int i = 0; i < totalRays; i++)
        {
            float t = totalRays == 1 ? 0.5f : (float)i / (totalRays - 1);
            float angle = Mathf.Lerp(minAngle, maxAngle, t);

            // direction relative to local forward
            Vector3 dir = Quaternion.Euler(0f, angle, 0f) * Vector3.forward;
            Vector3 worldDir = transform.TransformDirection(dir);

            if (Physics.SphereCast(origin, sphereRadius, worldDir, out RaycastHit hit, rayLength, rayLayerMask, QueryTriggerInteraction.Ignore))
            {
                if (hit.distance < nearestHitDistance)
                {
                    nearestHitDistance = hit.distance;
                    nearestHitPoint = hit.point;
                    // compute local x of hit relative to car (negative = left, positive = right)
                    Vector3 localHit = transform.InverseTransformPoint(hit.point);
                    nearestLocalX = localHit.x;
                }
            }
        }

        if (nearestHitDistance == float.MaxValue)
        {
            // nothing detected
            fuzzyThrottle = 1f;
            fuzzySteer = 0f;
            return;
        }

        // Compute fuzzy memberships (very simple triangular/shoulder shapes)
        // Near membership: 1 at 0..nearThreshold, falloff after
        float near = MembershipShoulderNear(nearestHitDistance, nearThreshold);
        float medium = MembershipTriangular(nearestHitDistance, nearThreshold * 0.8f, (nearThreshold + mediumThreshold) * 0.5f, mediumThreshold * 1.05f);
        float far = Mathf.Clamp01((nearestHitDistance - (mediumThreshold * 0.9f)) / (rayLength - mediumThreshold * 0.9f));

        // Throttle: heavily reduce when 'near' is high, slightly reduce when medium
        // final multiplier between minThrottleMultiplier and 1
        float throttleReduction = Mathf.Clamp01(near * 1.0f + medium * 0.45f); // weight near > medium
        fuzzyThrottle = Mathf.Lerp(1f, minThrottleMultiplier, throttleReduction);

        // Steering: steer away from obstacle. Use sign of localX to decide direction.
        // The stronger the near membership, the stronger the steer. Use medium for slight corrections.
        float steerAwayStrength = (near * 1.0f + medium * 0.4f);
        steerAwayStrength = Mathf.Clamp01(steerAwayStrength);
        float steerDir = Mathf.Sign(nearestLocalX); // +1 means obstacle is to right -> steer left (-)
        fuzzySteer = -steerDir * steerAwayStrength * steerStrength;
    }

    // Simple triangular membership (a,b,c)
    private float MembershipTriangular(float x, float a, float b, float c)
    {
        if (x <= a || x >= c) return 0f;
        if (x == b) return 1f;
        if (x > a && x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    // Shoulder membership for "near" (1 at distance<=threshold, then falls)
    private float MembershipShoulderNear(float x, float threshold)
    {
        if (x <= threshold * 0.6f) return 1f;
        if (x >= threshold * 1.6f) return 0f;
        // linear falloff
        float a = threshold * 0.6f;
        float c = threshold * 1.6f;
        return Mathf.Clamp01((c - x) / (c - a));
    }
}
