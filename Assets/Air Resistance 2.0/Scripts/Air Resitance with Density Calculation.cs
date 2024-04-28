using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#pragma warning disable 414
namespace AirResistance2
{
    [RequireComponent(typeof(Rigidbody))]
    public class AirResistanceWithDensityCalculation : MonoBehaviour
    {
        // Public fields for air density and variation that can be set from other scripts
        public float DensityVariation { get; set; }

        [Tooltip("Number of rays per unit of area. Higher numbers are more accurate but slower.")]
        [SerializeField] private float raycastDensity = 5.0f; // Rays/m^2
        [SerializeField] private bool useOverrideDensity = false; // Should we use override the air density value?
        [SerializeField] private bool useOverrideVariation = false; // Should we override the density variation value?
        [SerializeField] private bool debug = false; // Are we in debug mode?
        [SerializeField] private float overrideAirDensity; // Override air density value
        [SerializeField] private float overrideDensityVariation; // Override density variation value

        // Serialized field to display Mars density in the Inspector
        [SerializeField, Tooltip("The atmospheric density on Mars based on the current altitude.")]
        private float marsAtmosphericDensity;

        // Public property to access the Mars atmospheric density
        public float MarsAtmosphericDensity
        {
            get { return marsAtmosphericDensity; }
            private set { marsAtmosphericDensity = value; }
        }

        private new Transform transform; // Transform cache
        private new Rigidbody rigidbody; // Rigidbody cache
        private Vector3 scale; // Lossy scale of the object. Used for updating values later.
        private float radius; // Radius of the object
        private float increment; // Increment for placing rays, i.e. 1 / raycaseDensity
        private RaycastHit[] hitBuffer; // Buffer for raycast hits
        private float rigidbodyDrag; // Store for rigidbody.drag
        private bool isMarsSimulation = true; // Toggle for Mars simulation
        private float altitude = 0f; // Altitude of the object above Mars' surface

        private void Awake()
        {
            transform = GetComponent<Transform>();
            rigidbody = GetComponent<Rigidbody>();
            hitBuffer = new RaycastHit[16];
            rigidbodyDrag = rigidbody.drag;
            increment = 1.0f / raycastDensity;

            // Set our density and variation to the overrides. The inspector code handles setting this up
            if (useOverrideDensity)
            {
                MarsAtmosphericDensity = overrideAirDensity;
            }
            else if (isMarsSimulation)
            {
                MarsAtmosphericDensity = CalculateMarsAtmosphericDensity(altitude);
            }

            DensityVariation = useOverrideVariation ? overrideDensityVariation : 0f;
        }

        private void OnEnable()
        {
            rigidbody.drag = 0;
        }

        private void FixedUpdate()
        {
            if (rigidbody.IsSleeping() || GetMassNormalizedKE() < rigidbody.sleepThreshold) return;
            ValidateScale();

            if (isMarsSimulation)
            {
                // Update the MarsAtmosphericDensity property based on the object's altitude on Mars
                MarsAtmosphericDensity = CalculateMarsAtmosphericDensity(altitude);
                altitude = transform.position.y;
            }

            Vector3 up = -rigidbody.velocity.normalized;
            Vector3 forward = new Vector3(up.z, up.x, up.y);
            Vector3 right = Vector3.Cross(up, forward);
            float force = CalculateAirResistanceForce();

            Collider collider = GetComponent<Collider>();
            float colliderRadius = collider.bounds.extents.magnitude;
            float smallOffset = 0.01f;

            for (float x = -colliderRadius; x < colliderRadius; x += increment)
            {
                for(float z = -colliderRadius; z < colliderRadius; z += increment)
                {
                    Vector3 pos = (right * x + forward * z) + rigidbody.position - up * (colliderRadius + smallOffset);
                    Ray ray = new Ray(pos, up);
                    int len = Physics.RaycastNonAlloc(ray, hitBuffer);
                    RaycastHit hit = default;
                    for(int i = 0; i < len; i++)
                    {
                        hit = hitBuffer[i];
                        if(hit.transform != transform) continue;
                        
                        float variation = Random.Range(1.0f - DensityVariation, 1.0f + DensityVariation);
                        rigidbody.AddForceAtPosition(force * variation * up, hit.point);
                        break;
                    }

                    if (debug)
                    {
                        if(hit.transform == transform) Debug.DrawLine(pos, hit.point, Color.green);
                        else Debug.DrawLine(pos, pos + up * (colliderRadius + smallOffset), Color.red);
                    }
                }
            }
        }

        private void OnDisable()
        {
            rigidbody.drag = rigidbodyDrag;
        }

        private void OnValidate()
        {
            overrideAirDensity = Mathf.Max(0, overrideAirDensity);
            overrideDensityVariation = Mathf.Clamp(overrideDensityVariation, 0, 1);
        }

        private float CalculateAirResistanceForce()
        {
            float c = MarsAtmosphericDensity * increment * increment;
            float v2 = rigidbody.velocity.sqrMagnitude;
            return c * v2;
        }

        private float GetMassNormalizedKE()
        {
            float e = 0.5f * rigidbody.mass * rigidbody.velocity.sqrMagnitude;
            
            Vector3 inertia = rigidbody.inertiaTensor;
            Vector3 av = rigidbody.angularVelocity;
            e += 0.5f * inertia.x * av.x * av.x;
            e += 0.5f * inertia.y * av.y * av.y;
            e += 0.5f * inertia.z * av.z * av.z;

            return e / rigidbody.mass;
        }

        private void ValidateScale()
        {
            if(scale == transform.lossyScale) return;
            scale = transform.lossyScale;
            radius = scale.magnitude / 2.0f;
        }

        private float CalculateMarsAtmosphericDensity(float altitude)
        {
            const float T0Lower = -31f;
            const float T0Upper = -23.4f;
            const float lapseRateLower = -0.000998f;
            const float lapseRateUpper = -0.00222f;
            const float P0 = 0.699f;
            const float pressureRate = -0.00009f;
            const float R = 0.1921f;

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
    }
}
