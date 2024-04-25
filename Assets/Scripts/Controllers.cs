using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Controllers : MonoBehaviour
{
    public float speed = 10.0f;
    public float rotationSpeed = 100.0f;

    void Update()
    {
        // Get the input from the keyboard.
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        // Rotate the object around the x-axis.
        transform.Rotate(Vector3.right * rotationSpeed * Time.deltaTime * horizontalInput);

        // Move the object forward, backward, left, or right in world space.
        transform.Translate(transform.forward * speed * Time.deltaTime * verticalInput, Space.World);
    }
}

