using UnityEngine;

public class AirResistanceSimulation : MonoBehaviour
{
    // Aerodynamic data from SolidWorks simulation
    [Header("FLow Simulation Data")]
    [SerializeField] private float[] dragCoefficients = { 0.80859406f, 0.415849512f, 0.407191385f, 0.01619733f, 0.002796626f, -0.10439445f, -0.12627868f, -0.424713034f, 0.06799861f };
    [SerializeField] private float[] liftCoefficients = { 0.737856941f, 0.61829372f, 0.407191385f, 0.002516561f, 0.027966258f, 0.02270282f, -0.36785271f, -0.603465643f, -0.6172964f };
    [SerializeField] private float[] aoa = { 45f, 30f, 15f, -5f, -5f, -5f, -15f, -30f, -45f };

    [Header("Simulation Variables")]
    [SerializeField] private float airDensity;
    [SerializeField] private float angleOfAttack;
    [SerializeField] private float dragCoefficient;
    [SerializeField] private float liftCoefficient;
    [SerializeField] private float dragForce;
    [SerializeField] private float liftForce;
    
    private Rigidbody shuttleRigidbody;
    private float referenceArea;

    void Start()
    {
        shuttleRigidbody = GetComponent<Rigidbody>();
        referenceArea = CalculateReferenceArea();
    }

    void FixedUpdate()
    {
        ApplyAirResistance();
    }

    void ApplyAirResistance()
    {
        float altitude = shuttleRigidbody.position.y;
        float velocity = shuttleRigidbody.velocity.magnitude;

        // Calculate forces
        airDensity = CalculateMarsAtmosphericDensity(altitude);
        angleOfAttack = CalculateAngleOfAttack();
        dragCoefficient = InterpolateCoefficients(aoa, dragCoefficients, angleOfAttack);
        liftCoefficient = InterpolateCoefficients(aoa, liftCoefficients, angleOfAttack);
        dragForce = 0.5f * airDensity * velocity * velocity * dragCoefficient * referenceArea;
        liftForce = 0.5f * airDensity * velocity * velocity * liftCoefficient * referenceArea;

        // Apply forces
        Vector3 dragDirection = -shuttleRigidbody.velocity.normalized;
        Vector3 liftDirection = Vector3.Cross(shuttleRigidbody.velocity, Vector3.right).normalized; 
        // Assuming 'right' is the wing direction
        
        shuttleRigidbody.AddForce(dragDirection * dragForce, ForceMode.Force);
        shuttleRigidbody.AddForce(liftDirection * liftForce, ForceMode.Force);

        // Debugging lines
        Debug.DrawLine(transform.position, transform.position + dragDirection * dragForce, Color.red);
        Debug.DrawLine(transform.position, transform.position + liftDirection * liftForce, Color.green);
    }


    float CalculateAngleOfAttack()
    {
        Vector3 velocityVector = shuttleRigidbody.velocity.normalized;
        Vector3 forwardVector = transform.forward;
        float angleOfAttack = Mathf.Abs(Vector3.Angle(forwardVector, velocityVector) - 90f);
        return angleOfAttack;
    }

    float InterpolateCoefficients(float[] angles, float[] coefficients, float currentAngle)
    {
        // Handle exact matches first
        for (int i = 0; i < angles.Length; i++)
        {
            if (currentAngle == angles[i])
            {
                return coefficients[i];
            }
        }
        // Linear interpolation for in-between values
        for (int i = 0; i < angles.Length - 1; i++)
        {
            if (currentAngle >= angles[i] && currentAngle <= angles[i + 1])
            {
                float t = (currentAngle - angles[i]) / (angles[i + 1] - angles[i]);
                return coefficients[i] * (1 - t) + coefficients[i + 1] * t;
            }
        }
        // If the angle of attack is outside the range, use the closest boundary value
        return currentAngle <= angles[0] ? coefficients[0] : coefficients[angles.Length - 1];
    }


    float CalculateMarsAtmosphericDensity(float altitude)
    {
        // Calculation of atmospheric density on Mars
        // Constants for Mars' atmosphere
        const float T0Lower = -31f; // Base temperature at 0m altitude
        const float T0Upper = -23.4f; // Base temperature at 7000m altitude
        const float lapseRateLower = -0.000998f; // Temperature lapse rate below 7000m
        const float lapseRateUpper = -0.00222f; // Temperature lapse rate above 7000m
        const float P0 = 0.699f; // Base pressure
        const float pressureRate = -0.00009f; // Pressure rate
        const float R = 0.1921f; // Specific gas constant for Mars' atmosphere

        float temperature, pressure;
        if (altitude <= 7000)
        {
            temperature = T0Lower + lapseRateLower * altitude;
            pressure = P0 * Mathf.Exp(pressureRate * altitude);
        }
        else
        {
            temperature = T0Upper + lapseRateUpper * altitude;
            pressure = P0 * Mathf.Exp(pressureRate * altitude);
        }

        float density = pressure / (R * (temperature + 273.15f));
        return density;

    }

    float CalculateReferenceArea()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            return CalculateProjectedArea(meshFilter.mesh);
        }
        else
        {
            Debug.LogError("MeshFilter component not found! Reference area cannot be calculated.");
            return 0f;
        }
    }

    float CalculateProjectedArea(Mesh mesh)
    {
        float totalArea = 0f;
        Vector3 landerVelocityNormalized = shuttleRigidbody.velocity.normalized;
        
        foreach (var triangle in mesh.triangles)
        {
            // Get vertices of the triangle
            Vector3 v1 = mesh.vertices[mesh.triangles[triangle]];
            Vector3 v2 = mesh.vertices[mesh.triangles[triangle + 1]];
            Vector3 v3 = mesh.vertices[mesh.triangles[triangle + 2]];

            // Calculate the normal of the triangle
            Vector3 normal = Vector3.Cross(v2 - v1, v3 - v1).normalized;

            // Calculate the area of the triangle
            float triangleArea = Vector3.Cross(v1 - v2, v1 - v3).magnitude * 0.5f;

            // Project the area onto a plane perpendicular to the velocity vector
            float projectedArea = Mathf.Abs(Vector3.Dot(normal, landerVelocityNormalized)) * triangleArea;

            totalArea += projectedArea;
        }

        return totalArea;
    }
}
