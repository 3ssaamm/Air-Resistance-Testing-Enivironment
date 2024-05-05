using UnityEngine;

public class MarsLanderSimulation : MonoBehaviour
{
    [Header("Aerodynamic Data")]
    [SerializeField] private float[] dragCoefficients = { 0.80859406f, 0.415849512f, 0.407191385f, 0.01619733f, 0.002796626f, -0.10439445f, -0.12627868f, -0.424713034f, 0.06799861f };
    [SerializeField] private float[] liftCoefficients = { 0.737856941f, 0.61829372f, 0.407191385f, 0.002516561f, 0.027966258f, 0.02270282f, -0.36785271f, -0.603465643f, -0.6172964f };
    [SerializeField] private float[] aoa = { 45f, 30f, 15f, -5f, -5f, -5f, -15f, -30f, -45f };

    [Header("Simulation Variables")]
    [SerializeField] private float airDensity;
    [SerializeField] private float angleOfAttack;
    [SerializeField] private float dragCoefficient;
    [SerializeField] private float dragForce;
    [SerializeField] private float referenceArea;
    [SerializeField] private float minRotationSpeed = 0.1f; // Minimum rotation speed
    [SerializeField] private float maxRotationSpeed = 5f;   // Maximum rotation speed

    
    private Rigidbody landerRigidbody;
    

    void Start()
    {
        landerRigidbody = GetComponent<Rigidbody>();
        //referenceArea = CalculateReferenceArea();
    }

    void FixedUpdate()
    {
        ApplyForces();
        referenceArea = CalculateReferenceArea();
    }

    void ApplyForces()
    {
        Vector3 velocityVector = landerRigidbody.velocity;
        float velocityMagnitude = velocityVector.magnitude;

        // Calculate dynamic air density based on altitude
        airDensity = CalculateMarsAtmosphericDensity(landerRigidbody.position.y);
        
        // Calculate angle of attack and drag coefficient
        angleOfAttack = CalculateAngleOfAttack(velocityVector);
        dragCoefficient = InterpolateCoefficients(aoa, dragCoefficients, angleOfAttack);
        
        // Calculate dynamic drag force
        dragForce = 0.5f * airDensity * velocityMagnitude * velocityMagnitude * dragCoefficient * referenceArea;

        // Apply dynamic drag force
        Vector3 dragDirection = -velocityVector.normalized;
        landerRigidbody.AddForce(dragDirection * dragForce, ForceMode.Force);

        // Calculate dynamic rotation speed based on environmental factors and Rigidbody's angular drag
        float dynamicRotationSpeed = CalculateDynamicRotationSpeed(airDensity, velocityMagnitude, dragCoefficient, landerRigidbody.angularDrag);

        // Apply torque for rotation to align with the velocity vector
        Vector3 torqueDirection = Vector3.Cross(transform.forward, velocityVector).normalized;
        landerRigidbody.AddTorque(torqueDirection * dynamicRotationSpeed, ForceMode.Force);

        // Apply additional torque for pitch and yaw adjustments
        Vector3 pitchTorqueDirection = Vector3.Cross(transform.right, velocityVector).normalized;
        Vector3 yawTorqueDirection = Vector3.Cross(transform.up, velocityVector).normalized;
        landerRigidbody.AddTorque(pitchTorqueDirection * dynamicRotationSpeed, ForceMode.Force);
        landerRigidbody.AddTorque(yawTorqueDirection * dynamicRotationSpeed, ForceMode.Force);

        // Debugging lines
        Debug.DrawLine(transform.position, transform.position + dragDirection * dragForce, Color.red);
    }

    float CalculateDynamicRotationSpeed(float airDensity, float velocityMagnitude, float dragCoefficient, float angularDrag)
    {
        // Example calculation for dynamic rotation speed
        // Adjust the formula based on your simulation needs
        return Mathf.Clamp(airDensity * velocityMagnitude * dragCoefficient / (1 + angularDrag), minRotationSpeed, maxRotationSpeed);
    }


    float CalculateAngleOfAttack(Vector3 velocityVector)
    {
        Vector3 forwardVector = transform.forward;
        float angle = Vector3.Angle(forwardVector, velocityVector);

        // Additional logic to determine the sign of the angle
        float sign = Mathf.Sign(Vector3.Dot(transform.up, Vector3.Cross(forwardVector, velocityVector)));
        float signedAngle = angle * sign;
        return signedAngle;
    }


    float InterpolateCoefficients(float[] angles, float[] coefficients, float currentAngle)
    {
        // Linear interpolation for in-between values
        for (int i = 0; i < angles.Length - 1; i++)
        {
            if (currentAngle >= angles[i] && currentAngle <= angles[i + 1])
            {
                float t = (currentAngle - angles[i]) / (angles[i + 1] - angles[i]);
                return coefficients[i] * (1 - t) + coefficients[i + 1] * t;
            }
        }
        // If the angle is outside the range, use the closest boundary value
        return currentAngle <= angles[0] ? coefficients[0] : coefficients[angles.Length - 1];
    }

    float CalculateMarsAtmosphericDensity(float altitude)
    {
        // Mars atmospheric density calculation
        // Constants for Mars' atmosphere
        const float T0 = -31f; // Base temperature at 0m altitude
        const float lapseRate = -0.000998f; // Temperature lapse rate
        const float P0 = 0.699f; // Base pressure
        const float pressureRate = -0.00009f; // Pressure rate
        const float R = 0.1921f; // Specific gas constant for Mars' atmosphere

        float temperature = T0 + lapseRate * altitude;
        float pressure = P0 * Mathf.Exp(pressureRate * altitude);
        float density = pressure / (R * (temperature + 273.15f));
        return density;
    }

    float CalculateReferenceArea()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        if (meshFilter != null)
        {
            Mesh mesh = meshFilter.mesh;
            return CalculateProjectedArea(mesh);
        }
        else
        {
            Debug.LogError("MeshFilter component not found! Reference area cannot be calculated.");
            return 0f;
        }
    }


    float CalculateProjectedArea(Mesh mesh)
    {
        if (mesh == null)
        {
            Debug.LogError("Mesh is null. Cannot calculate projected area.");
            return 0f;
        }

        float totalArea = 0f;
        Vector3 landerVelocityNormalized = landerRigidbody.velocity.normalized;

        if (landerVelocityNormalized == Vector3.zero)
        {
            Debug.LogError("Velocity is zero. Cannot calculate projected area.");
            return 0f;
        }

        if (mesh.triangles.Length == 0)
        {
            Debug.LogError("Mesh triangles array is empty. Cannot calculate projected area.");
            return 0f;
        }

        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            Vector3 v1 = mesh.vertices[mesh.triangles[i]];
            Vector3 v2 = mesh.vertices[mesh.triangles[i + 1]];
            Vector3 v3 = mesh.vertices[mesh.triangles[i + 2]];

            Vector3 normal = Vector3.Cross(v2 - v1, v3 - v1).normalized;
            float triangleArea = Vector3.Cross(v1 - v2, v1 - v3).magnitude * 0.5f;
            float projectedArea = Mathf.Abs(Vector3.Dot(normal, landerVelocityNormalized)) * triangleArea;

            if (triangleArea == 0f)
            {
                Debug.LogWarning("Triangle area is zero. Check mesh data.");
            }

            totalArea += projectedArea;
        }

        if (totalArea == 0f)
        {
            Debug.LogError("Total projected area is zero. Check mesh data and scale.");
        }

        return totalArea;
    }


}
