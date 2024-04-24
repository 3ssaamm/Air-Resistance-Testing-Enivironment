using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DragWithOrientationAndAltitude : MonoBehaviour
{
    public float dragCoefficient = 0.24f; // Drag coefficient for a sphere
    public float referenceArea = 1.0f; // m^2 (Reference area for drag calculation)
    public float lowerAtmosphereHeight = 7000.0f; // Height at which the lower atmosphere ends and the upper atmosphere begins (m)
    public float m = 440;
    public float Diameter;
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

        float airDensity = pressure / (0.1921f * (temperature + 273.1f));

        // Calculate projected area based on object orientation
        float projectedArea = CalculateProjectedArea(velocity);

        // Calculate the drag force
        float dragForceMagnitude = 0.5f * dragCoefficient * airDensity * speed * speed * projectedArea;

        // Apply the drag force in the opposite direction of the velocity
        Vector3 dragForce = -velocity.normalized * dragForceMagnitude;

        rb.AddForce(dragForce);
    }

    float CalculateProjectedArea(Vector3 velocity)
    {
        // This uses a bounding box for simplicity, replace with your specific logic

        // Get world space points of the collider bounds
        Bounds bounds = collider.bounds;
        Vector3[] points = new Vector3[8];
        for (int i = 0; i < 8; i++)
        {
            points[i] = bounds.min;
            if ((i & 1) == 1) points[i].x = bounds.max.x;
            if ((i & 2) == 2) points[i].y = bounds.max.y;
            if ((i & 4) == 4) points[i].z = bounds.max.z;
        }

        // Project points onto a plane perpendicular to velocity
        Vector3 normal = velocity.normalized;
        Plane projectionPlane = new Plane(normal, points[0]); // Use any corner point

        float projectedArea = 0.0f;
        for (int i = 0; i < points.Length; i++)
        {
            // Distance from point to plane
            float distance = projectionPlane.GetDistanceToPoint(points[i]);

            // Project point onto plane
            Vector3 projectedPoint = points[i] - distance * normal;

            // Update projected area (assuming convex shape)
            projectedArea += Vector3.Cross(projectedPoint - points[i], points[(i + 1) % points.Length]).magnitude;
        }

        // Adjust projected area based on reference area
        projectedArea /= referenceArea;

        return Mathf.Clamp(projectedArea, 0.0f, 1.0f); // Ensure positive value
    }
}