using System.Collections;
using UnityEngine;


/// <summary>
/// HandController manages finger position, rotation and pressure from the controllers.
/// </summary>
public class HandController : MonoBehaviour {

    public eteeDevice device;
    public eteeAPI api;

    public bool isLeft;
    private Quaternion defaultHandPosition = Quaternion.identity;
    public bool calibrateHands = false;

    [Header("3D Model Finger Values")]
    // Finger values for the 3D hand model
    [Range(0f, 100f)]
    public float thumbFinger;                       // Used to estimate thumb finger joint angle
    [Range(0f, 100f)]
    public float indexFinger;                       // Used to estimate index finger joint angle
    [Range(0f, 100f)]         
    public float middleFinger;                      // Used to estimate middle finger joint angle
    [Range(0f, 100f)]
    public float ringFinger;                        // Used to estimate ring finger joint angle
    [Range(0f, 100f)]
    public float pinkyFinger;                       // Used to estimate pinky finger joint angle

    [Header("Finger Pull")]
    // Finger pull pressure values - Pull: first range of the pressure, representing light touch of fingers around the controller
    [Range(0f, 1f)]
    public float thumbPull;                       // Current thumb finger pull pressure value ( green bar in the UI )
    [Range(0f, 1f)]
    public float indexPull;                       // Current index finger pull pressure value ( green bar in the UI )
    [Range(0f, 1f)]
    public float middlePull;                      // Current middle finger pull pressure value ( green bar in the UI )
    [Range(0f, 1f)]
    public float ringPull;                        // Current ring finger pull pressure value ( green bar in the UI )
    [Range(0f, 1f)]
    public float pinkyPull;                       // Current pinky finger pull pressure value ( green bar in the UI )

    [Header("Finger Force")]
    // Finger force pressure values - Force: second range of the pressure, representing harder pressure of fingers on the controller
    [Range(0f, 1f)]
    public float thumbForce;                      // Current thumb finger force pressure value ( orange bar in the UI )
    [Range(0f, 1f)]
    public float indexForce;                      // Current index finger force pressure value ( orange bar in the UI )
    [Range(0f, 1f)]
    public float middleForce;                     // Current middle finger force pressure value ( orange bar in the UI )
    [Range(0f, 1f)]
    public float ringForce;                       // Current ring finger force pressure value ( orange bar in the UI )
    [Range(0f, 1f)]
    public float pinkyForce;                      // Current pinky finger force pressure value ( orange bar in the UI )

    // Previous values, needed for lerping
    float previousIndex;                            // Used to keep track of the previous Index finger value.           
    float previousMiddle;                           // Used to keep track of the previou Middle finger value.
    float previousRing;                             // Used to keep track of the previous Ring finger value.
    float previousPinky;                            // Used to keep track of the previous Pinky finger value.
    float previousThumb;                            // Used to keep track of the previous Thumb finger value.

    // Ranges
    float pullMin = 0f;
    float pullMax = 126f;

    float forceMin = 0f;
    float forceMax = 126f;

    static float forceContributionPct = 0.15f;
    float fingerMin = 0f;
    float fingerMax = 126f + forceContributionPct * 126f;

    [Header("Slider")]
    // Slider
    [Range(-140f, 140f)]
    public float sliderValue;
    public bool sliderTouch;
    public bool sliderUpButton;
    public bool sliderDownButton;

    [Header("Raw Controller Data")]
    public float rawIndex;                          // Raw Index finger value from the device - At the moment only upper electrodes are used so this value is calculated based on the raw upper value only.
    public float rawMiddle;                         // Raw Middle finger value from the device - At the moment only upper electrodes are used so this value is calculated based on the raw upper value only.
    public float rawRing;                           // Raw Ring finger value from the device - At the moment only upper electrodes are used so this value is calculated based on the raw upper value only.
    public float rawPinky;                          // Raw Pinky finger value from the device - At the moment only upper electrodes are used so this value is calculated based on the raw upper value only.

    public short rawIndexUpper;                     // Raw value from Index finger upper electrode.
    public short rawMiddleUpper;                    // Raw value from Middle finger upper electrode.
    public short rawRingUpper;                      // Raw value from Ring finger upper electrode.
    public short rawPinkyUpper;                     // Raw value from Pinky finger upper electrode.
    public short rawIndexLower;                     // Raw value from Index finger lower electrode - not used at the moment.
    public short rawMiddleLower;                    // Raw value from Middle finger lower electrode - not used at the moment.
    public short rawRingLower;                      // Raw value from Ring finger lower electrode - not used at the moment.
    public short rawPinkyLower;                     // Raw value from Pinky finger lower electrode -  not used at the moment.

    public short rawSliderButton;                      // Raw value from the slider button in the device.

    [Header("Other")]
    public Transform rotation;                  // External reference to the transform component.
    public Transform calibrationTransform;      // External reference to the transform componen for calibration.
    Quaternion q;                               // Temporal empty quaternion used to transform and move quaternions in several methods.

    Quaternion initialParentOrientation;        // Hand parent orientation - used to calibrate the rotation of the hand.

    public bool calibration = true;             // Whether calibration is enabled or disabled.

    //Filter 
    public float minimumChange;                     // Threshold used to detect when a pressure's change has happened on the device.

    public int tapStatus;                       // Whether the user is doing a tap or not.

    // Others
    public float toAdjust = 800f;                    // Fixed value used to adjust the raw signal. 

    public int battery;                             // Battery value displayed on the screen.       

    Animator anim;                                  // Hand animator component.

    private Renderer renderer;                      // Hand model renderer component.

    private string rightHandName = "RightHandModel";
    private string leftHandName = "LeftHandModel";
    
    public Hashtable MaxValues = new Hashtable();         // Maximun values for left hand.
    public Hashtable MinValues =  new Hashtable();        // Minimun values for right hand.


    void Start () {
        Init();
        defaultHandPosition = transform.localRotation;
        api.CalibrateFingers();
    }

    Coroutine calibrateHandsCoroutine;

    private void Update()
    {
        if (calibrateHands && device.enable)
        {
            // This is where the hand model rotations are set

            // Lerped the rotation animation to remove the jittering / shaking of hand models
            Quaternion newLocalRot = device.offsetToHand * device.quaternions;
            Quaternion oldLocalRot = transform.localRotation;
            float interp = 0.2f;

            Quaternion interpLocalRot = Quaternion.identity;
            interpLocalRot.x = Mathf.Lerp(oldLocalRot.x, newLocalRot.x, interp);
            interpLocalRot.y = Mathf.Lerp(oldLocalRot.y, newLocalRot.y, interp);
            interpLocalRot.z = Mathf.Lerp(oldLocalRot.z, newLocalRot.z, interp);
            interpLocalRot.w = Mathf.Lerp(oldLocalRot.w, newLocalRot.w, interp);

            transform.localRotation = interpLocalRot;
        }
        UpdateFingerData(isLeft? 0:1);
    }

    /// <summary>
    /// Updates each of the set devices fingers
    /// pull and force values retrieved from the
    /// data packet.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    private void UpdateFingerData(int device)
    {
        float[] pullData = api.GetAllFingersPull(device);
        float[] forceData = api.GetAllFingersForce(device);

        if (device == 0)
        {
            UpdateFingersData(pullData, forceData);
        }
        else
        {
            UpdateFingersData(pullData, forceData);
        }
    }

    /// <summary>
    /// Prevents hands from being calibrated
    /// and sets hands rotation to their default position
    /// </summary>
    public void StopHandMoving()
    {
        calibrateHands = false;
        transform.localRotation = defaultHandPosition;
    }

    /// <summary>
    /// Calibrates hands afte a second
    /// if device is enabled.
    /// </summary>
    public void CalibrateHand()
    {
        if (!device.enable)
        {
            return;
        }
        if (calibrateHandsCoroutine != null)
        {
            StopCoroutine(calibrateHandsCoroutine);
        }
        calibrateHandsCoroutine = StartCoroutine(CalibrateHandsCoroutine(1));
    }

    /// <summary>
    /// Initalises the Hand Controller class.
    /// </summary>
    private void Init() {

        // get animator component from the hand gameObject.
        anim = GetComponent<Animator>();

        // get initial orientation for calibration.
        initialParentOrientation = calibrationTransform.rotation;

        // get renderer component.
        if ( gameObject.name == leftHandName ) {
            renderer = GameObject.FindWithTag( "leftHandModel" ).GetComponent<Renderer>();
        } else if ( gameObject.name == rightHandName ) {
            renderer = GameObject.FindWithTag( "rightHandModel" ).GetComponent<Renderer>();
        }
    } 

    /// <summary>
    /// Checker for finger events. This method measures all
    /// changes in the fingers electrodes are read by the UI.
    /// This method is called every frame.
    /// </summary>
    private void listenForFingerEvents() {

        // check for changes in the index finger signal.
        if ( Mathf.Abs( indexFinger - previousIndex ) > minimumChange ) {
            anim.SetFloat( "IndexFinger", indexFinger );
            previousIndex = indexFinger;
        }

        // check for changes in the middle finger pressure. 
        if ( Mathf.Abs( middleFinger - previousMiddle ) > minimumChange ) {
            anim.SetFloat( "MiddleFinger", middleFinger );
            previousMiddle = middleFinger;
        }

        // check for changes in the ring finger pressure.
        if ( Mathf.Abs( ringFinger - previousRing ) > minimumChange ) {
            anim.SetFloat( "RingFinger", ringFinger);
            previousRing = ringFinger;
        }

        // check for changes in the pinky finger pressure.
        if ( Mathf.Abs( pinkyFinger - previousPinky ) > minimumChange ) {
            anim.SetFloat( "PinkyFinger", pinkyFinger);
            previousPinky = pinkyFinger;
        }

        // check for changes in the thumb finger pressure.
        if ( previousThumb != thumbFinger ) {
            anim.SetFloat( "ThumbFinger", thumbFinger );
            previousThumb = thumbFinger;
        }
     
    }

    /// <summary>
    /// Set when the hand model is renderer or not.
    /// Used to improve performance whent he user is
    /// out of the UI using computer controls.
    /// </summary>
    /// <param name="state">bool - true will render the hand models, otherwise the renderer component will be disabled</param>
    public void SetRenderer() {
       renderer.enabled = true;
    }

    /// <summary>
    /// Update hand fingers.
    /// </summary>
    /// <param name="data">array of floats - finger data coming from the device</param>
    public void UpdateFingersData( float[] pullData, float[] forceData) {
        //  check if the array contains data for all the fingers.

        if (pullData.Length == 5 ) {
            thumbPull = RemapValue(pullData[0], pullMin, pullMax, 0, 1);
            indexPull = RemapValue(pullData[1], pullMin, pullMax, 0, 1);
            middlePull = RemapValue(pullData[2], pullMin, pullMax, 0, 1);
            ringPull = RemapValue(pullData[3], pullMin, pullMax, 0, 1);
            pinkyPull = RemapValue(pullData[4], pullMin, pullMax, 0, 1);

            thumbForce = RemapValue(forceData[0], forceMin, forceMax, 0, 1);
            indexForce = RemapValue(forceData[1], forceMin, forceMax, 0, 1);
            middleForce = RemapValue(forceData[2], forceMin, forceMax, 0, 1);
            ringForce = RemapValue(forceData[3], forceMin, forceMax, 0, 1);
            pinkyForce = RemapValue(forceData[4], forceMin, forceMax, 0, 1);

            // The finger animations take data from both the pull and the force, in a way that a small portion
            // of the end finger curling is related to the force pressure range
            float newThumbFinger = RemapValue((thumbPull + thumbForce * forceContributionPct), fingerMin, fingerMax, 0, 80f);    // thumb finger curling range reduced
            float newIndexFinger = RemapValue((indexPull + indexForce * forceContributionPct), fingerMin, fingerMax, 0, 126f);
            float newMiddleFinger = RemapValue((middlePull + middleForce * forceContributionPct), fingerMin, fingerMax, 0, 126f);
            float newRingFinger = RemapValue((ringPull + ringForce * forceContributionPct), fingerMin, fingerMax, 0, 126f);
            float newPinkyFinger = RemapValue((pinkyPull + pinkyForce * forceContributionPct), fingerMin, fingerMax, 0, 126f);

            // Linear interpolation is used to create a smooth animation
            float interp = 0.2f;
            thumbFinger = Mathf.Lerp(thumbFinger, newThumbFinger, interp);
            indexFinger = Mathf.Lerp(indexFinger, newIndexFinger, interp);
            middleFinger = Mathf.Lerp(middleFinger, newMiddleFinger, interp);
            ringFinger = Mathf.Lerp(ringFinger, newRingFinger, interp);
            pinkyFinger = Mathf.Lerp(pinkyFinger, newPinkyFinger, interp);

            // update UI data.
            listenForFingerEvents();
        }

    }


    /// <summary>
    /// Remap values into a new range
    /// </summary>
    /// <param name="value">float - value to remap</param>
    /// <param name="low1">float - min value from the original range</param>
    /// <param name="high1">float - max value from the original range</param>.
    /// <param name="low2">float - min value from the new range that the value will be remapped to</param>
    /// <param name="high2">float - max value from the new range that the value will be remapped to</param>
    public float RemapValue( float value, float low1, float high1, float low2, float high2) {
        float newVal;
        if (value > high1)
        {
            newVal = 1.0f;
        }
        else
        {
            newVal = low2 + (value - low1) * (high2 - low2) / (high1 - low1);
        }
        return newVal;
    }

    /// <summary>
    /// Calibrates hands after the
    /// input amount of waiting time.
    /// </summary>
    /// <param name="waitingTime">int - how much time the couroutine should wait</param>
    /// <returns></returns>
    private IEnumerator CalibrateHandsCoroutine(int waitingTime)
    {
        int timePassed = waitingTime;
        calibrateHands = false;
        device.transform.localRotation = defaultHandPosition;
        Debug.Log("Reset");
        while (timePassed > 0)
        {
            timePassed -= 1;
            yield return new WaitForSecondsRealtime(1f);
            device.offsetToHand = device.quaternions;
        }
        device.offsetToHand = transform.localRotation * Quaternion.Inverse(device.offsetToHand);
        calibrateHands = true;
        Debug.Log("Hands Calibrated");
        calibrateHandsCoroutine = null;

    }

}
