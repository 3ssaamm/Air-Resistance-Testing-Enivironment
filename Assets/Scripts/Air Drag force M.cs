using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class dragwithorientation2 : MonoBehaviour
{
    public float dragCoefficient = 0.24f; // Drag coefficient for a sphere
    public float lowerAtmosphereHeight = 7000.0f; // Height at which the lower atmosphere ends and the upper atmosphere begins (m)
    public float m = 440;
    private Rigidbody rb;
    private Collider collider;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        collider = GetComponent<Collider>();
    }

    void FixedUpdate()
    {
        Vector3 velocity = rb.velocity;
        float speed = velocity.magnitude;

        // Calculate the air density as a function of altitude
        float height = transform.position.y;
        float temperature, pressure;

        if (height <= lowerAtmosphereHeight)
        {
            // Lower atmosphere
            temperature = -31 - 0.000998f * height;
            pressure = 0.699f * Mathf.Exp(-0.00009f * height);
        }
        else
        {
            // Upper atmosphere
            temperature = -23.4f - 0.00222f * height;
            pressure = 0.699f * Mathf.Exp(-0.00009f * height);
        }

        // Mars specific: Adjust air density calculation for Mars' primarily CO2 atmosphere
        float airDensity = pressure / (0.1921f * (temperature + 273.1f));

        // Calculate projected area based on object orientation
        float projectedArea = CalculateEffectiveArea(collider, velocity);

        // Calculate the drag force
        float dragForceMagnitude = 0.5f * dragCoefficient * airDensity * speed * speed * projectedArea;

        // Apply the drag force in the opposite direction of the velocity
        Vector3 dragForce = -velocity.normalized * dragForceMagnitude;

        // Apply the drag force in the direction of the rotation
        Vector3 rotation = transform.rotation.eulerAngles;
        Vector3 rotationForce = new Vector3(rotation.x, rotation.y, rotation.z) * dragForceMagnitude;

        rb.AddForce(dragForce + rotationForce);
    }

    float CalculateEffectiveArea(Collider collider, Vector3 velocity)
    {
        Mesh mesh = collider.GetComponent<MeshFilter>().mesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        float area = 0.0f;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 p1 = vertices[triangles[i]];
            Vector3 p2 = vertices[triangles[i + 1]];
            Vector3 p3 = vertices[triangles[i + 2]];

            // Calculate the normal of the triangle
            Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1).normalized;

            // Project the normal onto the velocity vector
            float projection = Mathf.Abs(Vector3.Dot(normal, velocity.normalized));

            // Calculate the area of the triangle
            float triangleArea = Vector3.Cross(p2 - p1, p3 - p1).magnitude * 0.5f;

            // Add the projected area of the triangle to the total area
            area += triangleArea * projection;
        }

        return area;
    }
}
