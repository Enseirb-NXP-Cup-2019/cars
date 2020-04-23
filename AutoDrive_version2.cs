using UnityEngine;
using System;
using System.Runtime.InteropServices;
using UnityEngine.UI;
using System.Collections;

public class AutoDrive : MonoBehaviour
{
    [Tooltip("Maximum steering angle of the wheels")]
	public float maxAngle = 50f;
	[Tooltip("Maximum torque applied to the driving wheels")]
	public float maxTorque = 300f;
	[Tooltip("Maximum brake torque applied to the driving wheels")]
	public float brakeTorque = 30000f;
	[Tooltip("If you need the visual wheels to be attached automatically, drag the wheel shape here.")]
	public GameObject wheelShape;

	[Tooltip("The vehicle's speed when the physics engine can use different amount of sub-steps (in m/s).")]
	public float criticalSpeed = 5f;
	[Tooltip("Simulation sub-steps when the speed is above critical.")]
	public int stepsBelow = 5;
	[Tooltip("Simulation sub-steps when the speed is below critical.")]
	public int stepsAbove = 1;

	[Tooltip("The vehicle's drive type: rear-wheels drive, front-wheels drive or all-wheels drive.")]
	public DriveType driveType;

    private WheelCollider[] m_Wheels;
    // The rendertexture of the car camera
    public RenderTexture carcamera;
    // The rigidbody of the car. Used to access to its speed 
    private Rigidbody rb;

    // Width and height of the image captured by the car camera
    private const int SIZEVIEW = 128;
    // The pixel array for the image captured
    private int[] pixels;
    // The texture captured by the car camera
    private Texture2D tex;

    // The y of the line of pixel observed, in the array of pixels. Its equation is y=LINE1
    private const int LINE1 = SIZEVIEW * SIZEVIEW / 2;
  ///  private const int LINE2 = (SIZEVIEW * SIZEVIEW / 4);
    private const int LINE2 = LINE1 + 2*SIZEVIEW;
    // x of leftmost pixel in the line when the car is in the middle of the lane
    private const int LEFTSEGMENT = 17; 
    // Size of the lane for the observed line
    private const int LANEWIDTH = 102-17;

    // Called once, at initialization of the scene
    void Start()
	{
        // Find the rigidbody of the car
        rb = GetComponent<Rigidbody>();
        // Find all the WheelColliders down in the hierarchy.
        m_Wheels = GetComponentsInChildren<WheelCollider>();
        for (int i = 0; i < m_Wheels.Length; ++i) 
		{
			var wheel = m_Wheels [i];
			// Create wheel shapes only when needed.
			if (wheelShape != null)
			{
				var ws = Instantiate (wheelShape);
				ws.transform.parent = wheel.transform;
			}
		}
        // Initialize texture and pixel array
        tex = new Texture2D(SIZEVIEW, SIZEVIEW);
        pixels = new int[SIZEVIEW * SIZEVIEW];
    }
    public int left = -1; // where is the left border of the lane
    public int gear = 0;// gear= 1: slow speed, gear = 2: high speed
    public const float SPEED1 = 3f, SPEED2 = 7f;

    // Called once each frame
    void Update()
    {
        // Dynamic parameters for driving the car
        float angle =0; // Direction of the front wheels. 0 = straight
        float torque = 0; // Angular acceleration of the wheels 
        float correction = 0; // Angle correction in order to drive in the middle of the lane
        float handBrake = 0; // handbrake > 0 is braking

        int i;
        int j;
        // Transform the image of the car camera into an array of pixels
        RenderTexture.active = carcamera;
        tex.ReadPixels(new Rect(0, 0, SIZEVIEW, SIZEVIEW), 0, 0);
        tex.Apply();
        Color32[] pix = tex.GetPixels32();
        for (i = 0; i < SIZEVIEW * SIZEVIEW; i++)
            pixels[i] = pix[i].r; // Only keep the red component

        /***** CODE FOR AUTOPILOT *****/
        // Find the leftmost black pixel at the line y=LINE1 in the image
        for (i = 0; i < SIZEVIEW; i++)
        {
            if (pixels[LINE1 + i] < 100) break;
        }
        for (j = 0; j < SIZEVIEW; j++)
        {
            if (pixels[LINE2+j] < 100) break;
        }
        if (i < SIZEVIEW)
        {
            if (j < SIZEVIEW && Math.Abs(i - j) < 1)
            {
                correction -= maxAngle*100;
            }
            else
            {
                // The pixel corresponds to the left border, or the right border of the lane 
                if (Math.Abs(i - left) < Math.Abs(i - left - LANEWIDTH)) left = i;
                else left = i - LANEWIDTH;

                // Find the correction to apply in order to be in the middle of the lane
                // To be in the middle of the lane: left = LEFTSEGMENT

                if (left < LEFTSEGMENT) correction -= maxAngle * (LEFTSEGMENT - left);
                else if (left > LEFTSEGMENT) correction += maxAngle * (left - LEFTSEGMENT);

                // Scaling factor from pixels to angle (hand tuned)
            }
            angle = correction *  0.009f;

            // Find the gear for the car 
            if (Mathf.Floor(correction) > 10) gear = 1; 
            else gear=2;
           
            // According to the gear and the speed, accelerate or brake
            if (gear == 1 && rb.velocity.magnitude < SPEED1) torque = maxTorque ;
            else if (gear == 1 && rb.velocity.magnitude > 1.2 * SPEED1) handBrake = brakeTorque * 0.1f;
            if (gear == 2 && rb.velocity.magnitude < SPEED2) torque = maxTorque;
            else if (gear == 2 && rb.velocity.magnitude > 1.2 * SPEED2) handBrake = brakeTorque * 0.1f;
        }
        /***** END OF AUTOPILOT *******/

        // Display of the car and its turning wheels
        m_Wheels[0].ConfigureVehicleSubsteps(criticalSpeed, stepsBelow, stepsAbove);
        foreach (WheelCollider wheel in m_Wheels)
        {
            // A simple car where front wheels steer while rear ones drive.
            if (wheel.transform.localPosition.z > 0)
                wheel.steerAngle = angle;

            wheel.motorTorque = torque;
            wheel.brakeTorque = handBrake;
            // Update visual wheels if any.
            if (wheelShape)
            {
                Quaternion q;
                Vector3 p;
                wheel.GetWorldPose(out p, out q);

                // Assume that the only child of the wheelcollider is the wheel shape.
                Transform shapeTransform = wheel.transform.GetChild(0);
                shapeTransform.position = p;
                shapeTransform.rotation = q;
            }
        }

    }

}
