using UnityEditorInternal;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// This component simulates the physics of a quadcopter.
/// It retrieves the current state of the Rigidbody, passes it to a flight
/// control system (FCS) placeholder, and applies the resulting forces and
/// torques to the drone's motors.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class QuadcopterController : MonoBehaviour
{

    [Tooltip("The 4 motor transforms. Order: FL, FR, RL, RR")]
    public Transform[] motorMounts;

    // --- Private Fields ---
    private Rigidbody rb;

    private Vector3 lastVelocity;
    /// <summary>
    /// A struct to hold the current physical state of the quadcopter.
    /// This is passed to the flight control system.
    /// </summary>
    public struct SensorData
    {
       public Vector3 gyroscope;
       public Vector3 accelerometer;
    };

    /// <summary>
    /// A struct to hold the output from the flight control system.
    /// This defines the forces to be applied by each motor.
    /// </summary>
    public struct ControlOutputs
    {
        // For a simple model, we can just use a single upward force per motor.
        // A more complex model could include torques for motor spin.
        public float[] motorForces; // One force value for each motor.
    }

    // --- Unity Methods ---

    /// <summary>
    /// Called when the script instance is being loaded.
    /// </summary>
    void Start()
    {
        // Get the Rigidbody component attached to this GameObject.
        rb = GetComponent<Rigidbody>();

        // Basic validation to ensure motor mounts are assigned.
        if (motorMounts == null || motorMounts.Length != 4)
        {
            Debug.LogError("Please assign 4 motor mount transforms in the inspector.");
            enabled = false; // Disable the script if not set up correctly.
        }
        lastVelocity = rb.linearVelocity;
    }

    /// <summary>
    /// Called every fixed-framerate frame. Use this for physics calculations.
    /// </summary>
    void FixedUpdate()
    {

        // 1. Get the current state of the Rigidbody.
        SensorData currentState = GetSensorData();

        // 2. Pass the state to the Flight Control System (FCS) to get control outputs.
        //    This is where your Teensy's logic would be replicated.
        

        // 3. Apply the forces and torques to the Rigidbody.
       // ApplyControlOutputs(outputs);
        Debug.Log(rb.GetAccumulatedForce());
    }

    // --- Custom Methods ---

    /// <summary>
    /// Gathers the current physical state of the quadcopter's Rigidbody.
    /// </summary>
    /// <returns>A struct containing the current state.</returns>
    private SensorData GetSensorData()
    {
        Vector3 localGravityReaction = transform.InverseTransformDirection(Physics.gravity);
        Vector3 velocityChange = (rb.linearVelocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = rb.linearVelocity;
        return new SensorData
        {
            gyroscope = rb.angularVelocity,
            accelerometer = velocityChange - localGravityReaction
        };
    }


    /// <summary>
    /// Applies the calculated forces from the control system to the motor positions.
    /// </summary>
    /// <param name="outputs">The control outputs from the FCS.</param>
    private void ApplyControlOutputs(ControlOutputs outputs)
    {
        if (outputs.motorForces == null || outputs.motorForces.Length != 4)
        {
            Debug.LogError("ControlOutputs did not contain 4 motor forces.");
            return;
        }

        // Apply an upward force at the position of each motor mount.
        for (int i = 0; i < motorMounts.Length; i++)
        {
            Transform motorMount = motorMounts[i];
            float force = outputs.motorForces[i];

            // Ensure force is not negative.
            force = Mathf.Max(0, force);

            // Apply the force in the local 'up' direction of the quadcopter.
            rb.AddForceAtPosition(transform.up * force, motorMount.position);

            // Optional: Visualize the forces in the editor
            Debug.DrawRay(motorMount.position, transform.up * force * 0.1f, Color.green);
        }
    }
}
