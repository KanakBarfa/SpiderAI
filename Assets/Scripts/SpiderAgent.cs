using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class SpiderAgent : Agent
{
    [Header("Spider Parts")]
    public Rigidbody2D body;

    //Target
    public Transform target;

    private Vector2 prevPos; // Previous position of the body
    //Aim Logic

    
    public Rigidbody2D[] upperLimbs; // size 3
    public Rigidbody2D[] lowerLimbs; // size 3
    public HingeJoint2D[] upperJoints;  // size 3
    private FixedJoint2D[] stickJoints;

    public float fieldRange = 10f;

    void Start()
    {
        stickJoints = new FixedJoint2D[3];
    }

    // === OBSERVATIONS ===
    public override void CollectObservations(VectorSensor sensor)
    {
        // Body state
        sensor.AddObservation((Vector2)body.transform.localPosition);
        sensor.AddObservation(body.rotation);
        sensor.AddObservation(body.linearVelocity);
        sensor.AddObservation(body.angularVelocity);

        // Joint states
        for (int i = 0; i < 3; i++)
        {
            sensor.AddObservation(upperJoints[i].jointAngle);
            sensor.AddObservation(upperLimbs[i].angularVelocity - body.angularVelocity);
        }

        // Stick states
        for (int i = 0; i < 3; i++)
            sensor.AddObservation(stickJoints[i] != null ? 1f : 0f);

        // Target state
        sensor.AddObservation((Vector2)target.localPosition);
    }

    // === ACTIONS ===
    public override void OnActionReceived(ActionBuffers actions)
    {
        var act = actions.ContinuousActions;

        // Joint angles (actions 0..5)
        for (int i = 0; i < 3; i++)
        {
            SetJointTarget(upperJoints[i], act[i]);
        }

        // Stick toggles (actions 6..8)
        var discreteAct = actions.DiscreteActions;

        for (int i = 0; i < 3; i++)
        {
            bool stick = discreteAct[i] == 1;
            ToggleStick(i, stick);
        }
    }

    // === HELPER FUNCTIONS ===
    private void SetJointTarget(HingeJoint2D joint, float normalized)
    {
        float targetAngle = Mathf.Lerp(0, 360, (normalized + 1f) / 2f);

        // Current angle
        float currentAngle = joint.jointAngle;
        float error = targetAngle - currentAngle;

        // PD control (proportional + damping) on angular velocity
        float kP = 5f; 
        float desiredAngularVel = error * kP;
        desiredAngularVel = Mathf.Clamp(desiredAngularVel, -20f, 20f);
        // Apply directly to Rigidbody2D
        joint.attachedRigidbody.angularVelocity = body.angularVelocity + desiredAngularVel;
    }

    private void ToggleStick(int i, bool stick)
    {
        if (stick && stickJoints[i] == null)
        {
            var fj = upperLimbs[i].gameObject.AddComponent<FixedJoint2D>();
            fj.connectedBody = null; // attach to world
            fj.enableCollision = false;
            stickJoints[i] = fj;
        }
        else if (!stick && stickJoints[i] != null)
        {
            Destroy(stickJoints[i]);
            stickJoints[i] = null;
        }
    }

    // === RESET ===
    public override void OnEpisodeBegin()
    {
        //Randomize body transform.localPosition
        body.transform.localPosition = new Vector2(Random.Range(-fieldRange, fieldRange), Random.Range(-fieldRange, fieldRange));
        body.linearVelocity = Vector2.zero;
        body.angularVelocity = 0f;
        body.rotation = Random.Range(0f, 360f);
        prevPos = body.transform.localPosition;
        // Reset limbs
        for (int i = 0; i < 3; i++)
        {
            upperLimbs[i].linearVelocity = Vector2.zero;
            upperLimbs[i].angularVelocity = 0f;
            upperLimbs[i].rotation = 0f;
            lowerLimbs[i].linearVelocity = Vector2.zero;
            lowerLimbs[i].angularVelocity = 0f;
            lowerLimbs[i].rotation = 0f;
        }

        // Reset stick state
        for (int i = 0; i < 3; i++)
        {
            if (stickJoints[i] != null) Destroy(stickJoints[i]);
            stickJoints[i] = null;
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var act = actionsOut.ContinuousActions;
        for (int i = 0; i < 3; i++)
            act[i] = 0f;

        // Example heuristic: random joint angles and random stick toggles
        for (int i = 0; i < 3; i++)
            act[i] = Random.Range(-1f, 1f);

        var discreteAct = actionsOut.DiscreteActions;
        for (int i = 0; i < 3; i++)
            discreteAct[i] = Random.value > 0.5f ? 1 : 0;
    }

    // === REWARDING ===
    private void FixedUpdate()
    {
        // Vector to target
        Vector2 toTarget = (Vector2)target.localPosition - (Vector2)body.transform.localPosition;
        float distanceToTarget = toTarget.magnitude;

        // Reward velocity toward target
        Vector2 velocity = ((Vector2)body.transform.localPosition - prevPos) / Time.fixedDeltaTime;
        float towardSpeed = Vector2.Dot(velocity, toTarget.normalized);
        AddReward(towardSpeed * 0.1f);  // scaled

        // Small time penalty
        AddReward(-0.02f);

        prevPos = body.transform.localPosition;
    }

}
