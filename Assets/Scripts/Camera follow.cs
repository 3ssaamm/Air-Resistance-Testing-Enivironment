using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; // The target object to follow
    private Vector3 offset;   // The offset distance between the camera and the target

    void Start()
    {
        // Set the offset to be the same as the camera's position
        offset = transform.position - target.position;
    }

    void LateUpdate()
    {
        // Update the camera's position to the target object's position plus the offset
        // This will make the camera follow the target without inheriting its rotation
        transform.position = target.position + offset;
        
        // Update the offset to be the same position and rotation of the camera itself
        offset = transform.position - target.position;
    }
}
