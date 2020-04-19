using UnityEngine;
using System;
using System.Runtime.InteropServices;
using UnityEngine.UI;
using System.Collections;

public class AutoDrive : MonoBehaviour
{
    [Tooltip("Maximum steering angle of the wheels")]
	public float maxAngle = 30f;
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
    // x of leftmost pixel in the line when the car is in the middle of the lane
    private const int LEFTSEGMENT = 17;
    // Size of the lane for the observed line
    private const int LANEWIDTH = 102-17;

    public GameObject cube;
    public GameObject car;
    private Vector3 posCube;
    private Vector3 posCar;
    public Vector3 tamp;
    public Vector3 speedCar;  // même si on a besoin que de 2 coordonnées Vector3  c'est pour avoir le même plan
    public Vector3 vecCubeCar;
    public Vector3 vecCarCube;
    public double angleCal;
    public double angleCompar;
    public double angleCompar2;
    public float sens;
    public double epsilon;
    public float coeff;

    // Called once, at initialization of the scene
    void Start()
	{
      cube = GameObject.Find("cube");
      car = GameObject.Find("carRoot");
      posCar = car.transform.position;
      posCube = cube.transform.position;
      tamp = posCar;
      speedCar = posCar - tamp;
      vecCubeCar.x = posCar.x - posCube.x;
      vecCubeCar.y = 0;
      vecCubeCar.z = posCar.z - posCube.z;



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
        if (i <SIZEVIEW)
        {
            // The pixel corresponds to the left border, or the right border of the lane
            if (Math.Abs(i - left) < Math.Abs(i - left - LANEWIDTH)) left = i;
            else left = i - LANEWIDTH;

            // Find the correction to apply in order to be in the middle of the lane
            // To be in the middle of the lane: left = LEFTSEGMENT
            if (left < LEFTSEGMENT) correction -= maxAngle * (LEFTSEGMENT - left);
            else if (left > LEFTSEGMENT) correction += maxAngle * (left - LEFTSEGMENT);
            // Scaling factor from pixels to angle (hand tuned)

            posCar = car.transform.position;
            posCube = cube.transform.position;
            print(tamp);
            print(posCar);
            //vecteur vitesse voiture
            speedCar = posCar - tamp;
            tamp = posCar;
            //vecteur voiture/obstacle
            vecCubeCar = posCar - posCube;
            vecCarCube = posCube - posCar;

            print(speedCar);
        //    print(vecCubeCar);
            // Comparer vecteur speed et CarCube si quasi même direction et sens = voiture fonce sur l'obstacle
        //    print(coeff);
            angleCal=Math.Acos((speedCar.x*vecCarCube.x+speedCar.z*vecCarCube.z)/(Math.Sqrt(Math.Pow(speedCar.x, 2)+Math.Pow(speedCar.z, 2)) * Math.Sqrt(Math.Pow(vecCarCube.x, 2) +Math.Pow(vecCarCube.z, 2))));
            angleCompar=Math.Acos((speedCar.x*1+speedCar.z*0)/Math.Sqrt(Math.Pow(speedCar.x, 2)+Math.Pow(speedCar.z, 2)));
            angleCompar2=Math.Acos((1*vecCarCube.x+0*vecCarCube.z)/Math.Sqrt(Math.Pow(vecCarCube.x, 2) +Math.Pow(vecCarCube.z, 2)));

            if (angleCompar - angleCompar2 < 0) {
              sens = -1;
            } else if (angleCompar2 - angleCompar < 0) {
              sens = 1;
            }
            if(Math.Abs(posCar.x-posCube.x+posCar.z-posCube.z) < 10 && Math.Abs(posCar.x-posCube.x+posCar.z-posCube.z) > 1 && angle<1) {
              correction += maxAngle/2 * sens;
              angle = correction*0.05f;
            } //si l'angle est trop faible, on tourne du coté ou on est le moins orienté vers le cube.
            angle = correction*0.01f;

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
