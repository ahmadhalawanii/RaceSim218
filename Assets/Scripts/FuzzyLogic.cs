using UnityEngine;


public class FuzzyLogic : MonoBehaviour
{
    [Header("Raycast (match the RayPerception settings)")]
    public int raysPerDirection = 6; 
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

    [HideInInspector] public float fuzzyThrottle = 1f;
    [HideInInspector] public float fuzzySteer = 0f;
    [HideInInspector] public float nearestHitDistance = 999f;
    [HideInInspector] public Vector3 nearestHitPoint = Vector3.zero;
    [HideInInspector] public float nearestForwardFactor = 0f; 

    void Awake()
    {
        fuzzyThrottle = 1f;
        fuzzySteer = 0f;
        nearestHitDistance = float.MaxValue;
        nearestForwardFactor = 0f;
    }

    public void SampleAndCompute()
    {
        int totalRays = raysPerDirection * 2 + 1;
        float minAngle = -maxRayDegrees;
        float maxAngle = maxRayDegrees;

        nearestHitDistance = float.MaxValue;
        nearestHitPoint = Vector3.zero;
        nearestForwardFactor = 0f;

        Vector3 origin = transform.position + Vector3.up * 0.5f;

        for (int i = 0; i < totalRays; i++)
        {
            float t = (totalRays == 1) ? 0.5f : (float)i / (totalRays - 1);
            float angle = Mathf.Lerp(minAngle, maxAngle, t);

            Vector3 localDir = Quaternion.Euler(0f, angle, 0f) * Vector3.forward;
            Vector3 worldDir = transform.TransformDirection(localDir);

            if (Physics.SphereCast(origin, sphereRadius, worldDir, out RaycastHit hit, rayLength, rayLayerMask, QueryTriggerInteraction.Ignore))
            {
                if (hit.distance < nearestHitDistance && hit.distance > minDistanceToConsider)
                {
                    nearestHitDistance = hit.distance;
                    nearestHitPoint = hit.point;

                    Vector3 toHitDir = (hit.point - origin).normalized;
                    nearestForwardFactor = Mathf.Clamp01(Vector3.Dot(transform.forward, toHitDir));

                }
            }
        }

        if (nearestHitDistance == float.MaxValue)
        {
            fuzzyThrottle = 1f;
            fuzzySteer = 0f;
            return;
        }

        float near = MembershipShoulderNear(nearestHitDistance, nearThreshold);
        float medium = MembershipTriangular(
            nearestHitDistance,
            nearThreshold * 0.8f,
            (nearThreshold + mediumThreshold) * 0.5f,
            mediumThreshold * 1.05f
        );

        float far = Mathf.Clamp01((nearestHitDistance - (mediumThreshold * 0.9f)) /
                                  (rayLength - mediumThreshold * 0.9f));

        float throttleReduction = Mathf.Clamp01(near * 1.0f + medium * 0.45f);
        fuzzyThrottle = Mathf.Lerp(1f, minThrottleMultiplier, throttleReduction);

        float steerAwayStrength = (near * 1.0f + medium * 0.4f);
        steerAwayStrength = Mathf.Clamp01(steerAwayStrength);

        float angleWeight = Mathf.Lerp(sideSteerWeight, 1f, nearestForwardFactor);
        steerAwayStrength *= angleWeight;

        Vector3 localHit = transform.InverseTransformPoint(nearestHitPoint);
        float steerDir = Mathf.Sign(localHit.x);
        fuzzySteer = -steerDir * steerAwayStrength * steerStrength;
    }

    private float MembershipTriangular(float x, float a, float b, float c)
    {
        if (x <= a || x >= c) return 0f;
        if (Mathf.Approximately(x, b)) return 1f;
        if (x > a && x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    private float MembershipShoulderNear(float x, float threshold)
    {
        float a = threshold * 0.6f;
        float c = threshold * 1.6f;

        if (x <= a) return 1f;
        if (x >= c) return 0f;

        return Mathf.Clamp01((c - x) / (c - a));
    }
}
