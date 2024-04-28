using UnityEngine;

public class InitialConditions : MonoBehaviour
{
    public Rigidbody rb;
    public Vector3 initialPosition;
    public Vector3 initialVelocity;

    public Vector3 currentVelocity;


    void Start()
    {
        // Set the initial position
        transform.position = initialPosition;

        // Ensure there's a Rigidbody component attached to the object
        rb = GetComponent<Rigidbody>();

        // Set the initial velocity
        rb.velocity = initialVelocity;
    }

    void Update()
    {
        // Update the currentVelocity with the Rigidbody's velocity each frame
        currentVelocity = rb.velocity;
    }
}
