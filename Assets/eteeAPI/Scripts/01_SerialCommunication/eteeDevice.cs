using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using AHRS;
using IMUPipeline;
/// <summary>
/// Gathers values from device data packet using UpdateValue() functions.
/// </summary>
public class eteeDevice : MonoBehaviour {

    [Header("Serial Communication")]
    public CSharpSerial stream;

    [Header("Handedness")]
    public bool isLeft = false;

    [Header("Battery Status")]
    public float battery;                                       // Battery value.
    public bool chargingInProgress;
    public bool chargingComplete;

    [Header("Finger Data")]
    public float[] fingerPullData = new float[5];               // Finger pull pressure array.
    public float[] fingerForceData = new float[5];              // Finger force pressure array.
    public bool[] fingerTouchedData = new bool[5];              // Finger touched value array.
    public bool[] fingerClickedData = new bool[5];              // Finger clicked value array.

    public Tuple<float, float> thumb;                            // Pressure values for thumb finger.
    public Tuple<float, float> index;                            // Pressure values for index finger.
    public Tuple<float, float> middle;                           // Pressure values for middle finger.
    public Tuple<float, float> ring;                             // Pressure values for ring finger.
    public Tuple<float, float> pinky;                            // Pressure values for pinky finger.

    [Header("System Button")]
    public bool systemButtonPressed;

    [Header("Slider")]
    public float sliderValue;                                   // Slider Y button value.
    public bool sliderButton;
    public bool sliderUpButton;
    public bool sliderDownButton;

    [Header("Trackpad")]
    public Vector2 trackpadCoordinates;                         // Trackpad vector2 coordinates.
    public Tuple<float, float> trackpadPressures;               // Trackpad vector2 pressures.
    public bool trackpadTouched;
    public bool trackpadClicked;

    [Header("Trackpad Taps")]
    public bool trackpadTapped;

    public bool tap;                                            // Tap value.
    public bool doubleTap;
    public Tuple<bool, bool> taps;
    private bool prevLeftTap;
    private bool prevRightTap;
    private bool[] leftTapArr;
    private bool[] rightTapArr;
    private float pastLeftTapTime;
    private float pastRightTapTime;
    private float tapTimeThreshold = 0.03f;
    private int tapArrSize = 17;

    [Header("Tracker")]
    public bool trackerConnected;                               // Tracker connection value
    public bool proxTouched;                                    // Proximity sensor touch value
    public bool proxClicked;                                    // Proximity sensor click value
    public float proxValue;                                     // Proximity sensor analog value 

    [Header("Raw IMU")]
    public Vector3 accelerometer;                               // Accelerometer data.
    public Vector3 accelerometerRawInt16;                       // Raw int16 accelerometer data (for pipeline).

    public Vector3 gyroscope;                                   // Gyroscope data.
    public bool gyroCalibrated = false;
    public bool gyroCalibrationDone = false;
    private int gyroCalibratingSamples = 700;
    public Vector3 gyroscopeOffsetValues = Vector3.zero;

    public Vector3 gyroscopeRawInt16;                             // Raw int16 gyroscope data (for pipeline).

    public Vector3 magnetometer;                                // Magnetometer data.
    public bool magCalibrated = true;
    public float[] magCalibration = new float[3];
    public bool magCalibrationDone = false;
    private int magCalibratingSamples = 700;
    public Vector3 magOffsetValues = Vector3.zero;
    private float[] magMinimum = new float[3];
    private float[] magMaximum = new float[3];
    public float[] magOffset = new float[] { 0, 0, 0 };
    private float[] magScale = new float[] { 1f, 1f, 1f };

    [Header("Device Rotation")]
    public Quaternion quaternions;                              // Quaternion values for rotation.
    public Quaternion offsetToHand = Quaternion.identity;
    public Vector3 anglesOffset;
    public Vector3 euler;

    private int samplesTaken = 0;
    public float samplePeriod = 100f;
    public float beta = 0.0315f;
    private MadgwickAHRS madgwickAHRS;

    // Sensor conversion constants
    private const float ACCEL_SCALE = 4.0f / 32768.0f;
    private const float GYRO_SCALE_DPS = 2000.0f / 32768.0f;
    private const float GYRO_DPS_TO_RPS = Mathf.PI / 180.0f;

    // Per-device gyro bias estimator with ZUPT (replaces old gyro offset)
    private GyroBiasEstimator gyroBiasEstimator;

    // Shared yaw filter for drift-free yaw (set by CSharpSerial)
    [HideInInspector]
    public AnchoredYawFilter sharedYawFilter;

    // Magnetometer calibrator for hard-iron correction + reliability detection
    private MagCalibrator magCalibrator;
    private Vector3 magCorrected;
    private bool magIsReliable;

    // Per-device pitch/roll freeze state
    private float lastStablePitch;
    private float lastStableRoll;
    private bool hasPitchRollSnapshot = false;

    // Persistent yaw offset for drift correction
    private float persistentYawOffset = 0f;

    // IMU diagnostic counter
    private int imuDiagCounter = 0;

    // Snapshot-based magnetic yaw drift corrector
    private MagSnapshotYawCorrector magYawCorrector;

    // Accumulated yaw correction offset (degrees) — applied at output stage, NOT written back to AHRS
    private float yawCorrectionAccum = 0f;

    // Flag: AHRS quaternion needs gravity initialization from first accel reading
    private bool ahrsNeedsGravityInit = true;

    // Packet filtering
    private bool handIdentityEstablished = false;
    private int misroutedPacketCount = 0;

    public float roll;
    public float pitch;
    public float yaw;

    [Header("Gestures")]
    public bool squeeze;                                        // Squeeze gesture.
    public bool gripTouch;
    public bool gripClick;
    public Tuple<float, float> gripPressures;

    public bool pointIndependent;                               // Point Independent gesture boolean
    public float pointIndependentAnalog;                        // Point Independent gesture analog

    public bool pointExcludeTrackpad;                           // Point Exclude Trackpad gesture boolean
    public float pointExcludeTrackpadAnalog;                    // Point Exclude Trackpad gesture analog

    public bool pinchTrackpad;                                  // Pinch trackpad gesture boolean
    public float pinchTrackpadAnalog;                             // Pinch trackpad gesture analog

    public bool pinchThumbFinger;                               // Pinch thumbFinger gesture boolean
    public float pinchThumbFingerAnalog;                          // Pinch thumbFinger gesture analog

    [Header("Flags")]
    public bool enable = false;
    public bool handSmoothing = true;


    // start is called at the first frame before update.
    private void Start()
    {
        ResetValues();
        prevLeftTap = false;
        prevRightTap = false;
        leftTapArr = new bool[tapArrSize];
        rightTapArr = new bool[tapArrSize];
        pastLeftTapTime = Time.time;
        pastRightTapTime = Time.time;

        madgwickAHRS = new MadgwickAHRS(1 / samplePeriod, beta);
        this.gyroBiasEstimator = new GyroBiasEstimator();
        this.magCalibrator = new MagCalibrator();
        this.magYawCorrector = new MagSnapshotYawCorrector();
        this.magCorrected = Vector3.zero;
        this.magIsReliable = false;
    }


    /// <summary>
    /// Update values from raw byte data.
    /// This method is called by the C# Serial Reader.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    public void UpdateValuesFromController(byte[] serialBuffer)
    {
        bool packetIsLeft = !IsBitSet(serialBuffer[11], 3);

        // Guard: after first valid packet, reject any packet with wrong L/R bit.
        // This prevents corrupted/mis-routed packets from poisoning AHRS state.
        if (handIdentityEstablished && packetIsLeft != isLeft)
        {
            misroutedPacketCount++;
            if (misroutedPacketCount <= 5 || misroutedPacketCount % 100 == 0)
            {
                string expected = isLeft ? "L" : "R";
                string got = packetIsLeft ? "L" : "R";
                Debug.LogWarning($"[eteeDevice-{expected}] Mis-routed packet #{misroutedPacketCount} " +
                                 $"(got={got}). Discarding to protect AHRS.");
            }
            return;
        }

        enable = true;
        isLeft = packetIsLeft;
        if (!handIdentityEstablished)
        {
            handIdentityEstablished = true;
            Debug.Log($"[eteeDevice] Hand identity established: {(isLeft ? "LEFT" : "RIGHT")}");
        }

        // update finger data.
        UpdateFingers(serialBuffer);

        // update battery value.
        UpdateBatteryValue(serialBuffer);

        // update charging status
        UpdateCharging(serialBuffer);

        // update slider value.
        UpdateSliderButtonValue(serialBuffer);

        // update tracker value
        UpdateTrackerConnection(serialBuffer);

        UpdateTrackerValues(serialBuffer);

        // update trackpad values.
        UpdateTrackPadValues(serialBuffer);

        // update tap value.
        UpdateTapValues();

        // update system button value.
        UpdateSystemButton(serialBuffer);

        // update accelerometer values.
        UpdateAccelerometerValues(serialBuffer);

        // update accelerometer values.
        UpdateGyroscopeValues(serialBuffer);

        // update accelerometer values.
        UpdateMagnetometerValues(serialBuffer);

        // update squeeze gesture.
        UpdateSqueezeGesture(serialBuffer);

        // update point A gesture value.
        UpdatePointIndependentGesture(serialBuffer);

        // update point B gesture value.
        UpdatePointExcludeTrackpadGesture(serialBuffer);

        // update pinch trackpad gesture value.
        UpdatePinchTrackpadGesture(serialBuffer);

        // update pinch finger and thumb gesture value.
        UpdatePinchThumbFingerGesture(serialBuffer);

        EstimateIMUOrientation();

        CalibrateGyro();
    }

    // ==================================== Controller Values ====================================
    /// <summary>
    /// Update fingers value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateFingers(byte[] serialBuffer)
    {
        // update individual fingers data.
        float thumbPullValue = SelectBits(serialBuffer[1], 7, 1);
        float thumbForceValue = SelectBits(serialBuffer[18], 7, 1);
        bool thumbClicked = IsBitSet(serialBuffer[0], 3);
        bool thumbTouched = IsBitSet(serialBuffer[1], 0);
        thumb = new Tuple<float, float>(thumbPullValue, thumbForceValue);

        float indexPullValue = SelectBits(serialBuffer[2], 7, 1);
        float indexForceValue = SelectBits(serialBuffer[19], 7, 1);
        bool indexClicked = IsBitSet(serialBuffer[0], 4);
        bool indexTouched = IsBitSet(serialBuffer[2], 0);
        index = new Tuple<float, float>(indexPullValue, indexForceValue);

        float middlePullValue = SelectBits(serialBuffer[3], 7, 1);
        float middleForceValue = SelectBits(serialBuffer[20], 7, 1);
        bool middleClicked = IsBitSet(serialBuffer[0], 5);
        bool middleTouched = IsBitSet(serialBuffer[3], 0);
        middle = new Tuple<float, float>(middlePullValue, middleForceValue);

        float ringPullValue = SelectBits(serialBuffer[4], 7, 1);
        float ringForceValue = SelectBits(serialBuffer[21], 7, 1);
        bool ringClicked = IsBitSet(serialBuffer[0], 6);
        bool ringTouched = IsBitSet(serialBuffer[4], 0);
        ring = new Tuple<float, float>(ringPullValue, ringForceValue);

        float pinkyPullValue = SelectBits(serialBuffer[5], 7, 1);
        float pinkyForceValue = SelectBits(serialBuffer[22], 7, 1);
        bool pinkyClicked = IsBitSet(serialBuffer[0], 7);
        bool pinkyTouched = IsBitSet(serialBuffer[5], 0);
        pinky = new Tuple<float, float>(pinkyPullValue, pinkyForceValue);

        // update fingers array
        fingerPullData = new float[] { thumbPullValue, indexPullValue, middlePullValue, ringPullValue, pinkyPullValue };
        fingerForceData = new float[] { thumbForceValue, indexForceValue, middleForceValue,ringForceValue, pinkyForceValue };
        fingerClickedData = new bool[] { thumbClicked, indexClicked, middleClicked, ringClicked, pinkyClicked };
        fingerTouchedData = new bool[] { thumbTouched, indexTouched, middleTouched, ringTouched, pinkyTouched };
    }


    /// <summary>
    /// Update battery value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateBatteryValue(byte[] serialBuffer)
    {
        // battery values is located in the byte 14 in the serial buffer array.
        battery = SelectBits(serialBuffer[12], 7, 1);
    }

    /// <summary>
    /// Update slider button pressed value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateSliderButtonValue(byte[] serialBuffer)
    {
        sliderValue = SelectBits(serialBuffer[9], 7, 1);
        sliderButton = IsBitSet(serialBuffer[9], 0);
        sliderUpButton = IsBitSet(serialBuffer[11], 5);
        sliderDownButton = IsBitSet(serialBuffer[11], 6);
    }

    /// <summary>
    /// Checks if the selected controller has an eteeTracker connected.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    private void UpdateTrackerConnection(byte[] serialBuffer)
    {
        trackerConnected = IsBitSet(serialBuffer[11], 2);
    }

    /// <summary>
    /// Updates proximity values for tracker.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    private void UpdateTrackerValues(byte[] serialBuffer)
    {
        proxClicked = IsBitSet(serialBuffer[11], 1);
        proxTouched = IsBitSet(serialBuffer[8], 0);
        proxValue = SelectBits(serialBuffer[8], 7, 1);
    }

    /// <summary>
    /// Update tap action value.
    /// </summary>
    /// <returns>void</returns>
    private void UpdateTapValues()
    {
        // This parameter is not in the data packet anymore, so calculate its value in the software layer.
        trackpadTapped = trackpadTouched;

        // Left device taps
        if (isLeft)
        {
            // Calculate tap
            if (trackpadTapped != prevLeftTap)
            {
                if (trackpadTapped)  // TP touch just changed from False to True
                {
                    tap = true;
                    prevLeftTap = true;
                }
                else if (!trackpadTapped)  // TP touch just changed from True to False
                {
                    tap = false;
                    prevLeftTap = false;
                }
            }

            // Updates the state of the tap every 200ms
            float currentTime = Time.time;
            if ((currentTime - pastLeftTapTime) > tapTimeThreshold)
            {
                pastLeftTapTime = Time.time;
                leftTapArr = NewTapArray(0, tap);
            }

            // Check for double taps
            int numTaps = CheckNumberTaps(0);
            if (numTaps > 1)
            {
                doubleTap = true;
            }
            else
            {
                doubleTap = false;
            }

            if (doubleTap)
            {
                leftTapArr = new bool[tapArrSize];
            }
        }

        // Right device taps
        else if (!isLeft)
        {
            // Calculate tap
            if (trackpadTapped != prevRightTap)
            {
                if (trackpadTapped)  // TP touch just changed from False to True
                {
                    tap = true;
                    prevRightTap = true;
                }
                else if (!trackpadTapped)  // TP touch just changed from True to False
                {
                    tap = false;
                    prevRightTap = false;
                }
            }

            // Updates the state of the tap every 200ms
            float currentTime = Time.time;
            if ((currentTime - pastRightTapTime) > tapTimeThreshold)
            {
                pastRightTapTime = Time.time;
                rightTapArr = NewTapArray(1, trackpadTouched);
            }

            // Check for double taps
            int numTaps = CheckNumberTaps(1);
            if (numTaps > 1)
            {
                doubleTap = true;
            }
            else
            {
                doubleTap = false;
            }

            if (doubleTap)
            {
                rightTapArr = new bool[tapArrSize];
            }
        }

        taps = new Tuple<bool, bool>(tap, doubleTap);
    }

    /// <summary>
    /// Updates bool for system button being pressed
    /// from the data packet.
    /// </summary>
    /// <param name="serialbuffer">byte array - array data from the device</param>
    private void UpdateSystemButton(byte[] serialbuffer)
    {
        systemButtonPressed = IsBitSet(serialbuffer[0], 0);
    }

    /// <summary>
    /// Adds the new value to the end of the tap array and shifts the prevuious elements to the left in a way that the previous element with index 0 disappears.
    /// </summary>
    /// <param name="device">int - whether the controller is a right (1) or left (0) controller</param>
    /// <param name="newValue">bool - new value to add at the end of the new tap array</param>
    /// <returns>bool[]</returns>
    private bool[] NewTapArray(int device, bool newValue)
    {
        bool[] oldArray = device == 0 ? leftTapArr : rightTapArr;
        bool[] newArray = new bool[leftTapArr.Length];

        for (int i = 0; i < (oldArray.Length - 1); i++)
        {
            newArray[i] = oldArray[i + 1];
        }

        newArray[oldArray.Length - 1] = newValue;

        return newArray;
    }

    /// <summary>
    /// Returns the number of taps in a tap array.
    /// </summary>
    /// <param name="device">int - whether the controller is a right (1) or left (0) controller</param>
    /// <returns>bool[]</returns>
    private int CheckNumberTaps(int device)
    {
        bool[] arrToCheck = device == 0 ? leftTapArr : rightTapArr;
        int numTaps = 0;
        for (int i = 0; i < (arrToCheck.Length - 1); i++)
        {
            if (arrToCheck[i] != arrToCheck[i + 1])
            {
                if (arrToCheck[i + 1])  // TP touch just changed from False to True
                {
                    numTaps++;
                }
            }
        }
        return numTaps;
    }

    /// <summary>
    /// Update trackPad coordinates.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateTrackPadValues(byte[] serialBuffer)
    {
        byte xValue = unchecked(serialBuffer[6]);   // x and y-axis coordinates
        byte yValue = unchecked(serialBuffer[7]);

        float pullValue = SelectBits(serialBuffer[13], 7, 1);
        float forceValue = SelectBits(serialBuffer[17], 7, 1);

        trackpadCoordinates = new Vector3(xValue, yValue);
        trackpadPressures = new Tuple<float, float>(pullValue, forceValue);
        trackpadClicked = IsBitSet(serialBuffer[0], 1);
        trackpadTouched = IsBitSet(serialBuffer[0], 2);
    }


    // ==================================== Rotation ====================================

    /// <summary>
    /// Measures controller orientation quaternions using multi-stage IMU pipeline:
    /// Stage 1: Raw int16 → physical units + ZUPT gyro bias correction
    /// Stage 1.5: Gravity-based AHRS initialization on first frame
    /// Stage 2: 6-axis Madgwick AHRS with adaptive beta
    /// Stage 3: Snapshot-based magnetic yaw drift correction
    /// </summary>
    private void EstimateIMUOrientation()
    {
        // ====================================================================
        // STAGE 1: Convert raw int16 → physical units + bias correction
        // ====================================================================
        Vector3 accelG = accelerometerRawInt16 * ACCEL_SCALE;
        Vector3 gyroDps = gyroscopeRawInt16 * GYRO_SCALE_DPS;

        // Dynamic gyro bias correction with ZUPT (returns zero when static)
        Vector3 gyroCorrDps = gyroBiasEstimator.Update(gyroDps, accelG);

        // Convert corrected gyro to rad/s for Madgwick
        Vector3 gyroRps = gyroCorrDps * GYRO_DPS_TO_RPS;

        // ====================================================================
        // STAGE 1.5: Initialize AHRS from gravity on first valid accel frame
        // ====================================================================
        // AHRS defaults to (1,0,0,0) which has NO gravity alignment.
        // Fix: compute initial quaternion from accel so pitch/roll are
        // correct from frame 1. Yaw stays arbitrary (no heading ref yet).
        if (ahrsNeedsGravityInit && accelG.sqrMagnitude > 0.5f)
        {
            Vector3 a = accelG.normalized;
            float dot = a.z;
            float qw, qx, qy, qz;

            if (dot < -0.999f)
            {
                qw = 0f; qx = 1f; qy = 0f; qz = 0f;
            }
            else
            {
                qw = 1f + dot;
                qx = a.y;
                qy = -a.x;
                qz = 0f;
            }

            float qNorm = Mathf.Sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            if (qNorm > 0.001f)
            {
                qNorm = 1f / qNorm;
                madgwickAHRS.Quaternion[0] = qw * qNorm;
                madgwickAHRS.Quaternion[1] = qx * qNorm;
                madgwickAHRS.Quaternion[2] = qy * qNorm;
                madgwickAHRS.Quaternion[3] = qz * qNorm;
                string hand = isLeft ? "L" : "R";
                Debug.Log($"[AHRS-{hand}] Gravity-init from accel: ({a.x:F3}, {a.y:F3}, {a.z:F3})");
            }
            ahrsNeedsGravityInit = false;
        }

        // ====================================================================
        // STAGE 2: 6-axis Madgwick AHRS with adaptive beta
        // ====================================================================
        float gyroMag = gyroCorrDps.magnitude;
        float adaptiveBeta;

        if (gyroCorrDps.sqrMagnitude < 0.001f)
        {
            if (gyroBiasEstimator.IsSettling)
                adaptiveBeta = 0.003f;   // Settling: gentle gravity correction
            else
                adaptiveBeta = 0f;       // Fully static: freeze
        }
        else if (gyroMag < 50f)
        {
            adaptiveBeta = beta;
        }
        else
        {
            adaptiveBeta = beta * Mathf.Clamp01(1.0f - (gyroMag - 50f) / 250f);
            adaptiveBeta = Mathf.Max(adaptiveBeta, 0.001f);
        }

        madgwickAHRS.Beta = adaptiveBeta;
        madgwickAHRS.SamplePeriod = 1f / samplePeriod;
        madgwickAHRS.UpdateRelative(gyroRps.x, gyroRps.y, gyroRps.z,
            accelG.x, accelG.y, accelG.z);

        float q0 = madgwickAHRS.Quaternion[0]; // w
        float q1 = madgwickAHRS.Quaternion[1]; // x
        float q2 = madgwickAHRS.Quaternion[2]; // y
        float q3 = madgwickAHRS.Quaternion[3]; // z

        // ====================================================================
        // STAGE 3: Snapshot-based magnetic yaw correction
        // ====================================================================
        imuDiagCounter++;
        string handLabel0 = isLeft ? "L" : "R";
        if (magCalibrator.IsMagnetometerPresent && magCalibrator.IsCalibrated)
        {
            float pitchRad = Mathf.Asin(Mathf.Clamp(-2f * (q1 * q3 - q0 * q2), -1f, 1f));
            float rollRad  = Mathf.Atan2(2f * (q0 * q1 + q2 * q3), 1f - 2f * (q1 * q1 + q2 * q2));

            float cosPitch = Mathf.Cos(pitchRad);
            float sinPitch = Mathf.Sin(pitchRad);
            float cosRoll  = Mathf.Cos(rollRad);
            float sinRoll  = Mathf.Sin(rollRad);

            float mx = magCorrected.x;
            float my = magCorrected.y;
            float mz = magCorrected.z;
            float magX = mx * cosPitch + mz * sinPitch;
            float magY = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;

            float magHeading = Mathf.Atan2(-magY, magX) * Mathf.Rad2Deg;
            float ahrsYaw    = Mathf.Atan2(2f * (q0 * q3 + q1 * q2),
                                           1f - 2f * (q2 * q2 + q3 * q3)) * Mathf.Rad2Deg;

            string handLabel = isLeft ? "L" : "R";
            float accelDev   = Mathf.Abs(accelG.magnitude - 1f);

            float yawCorrection = magYawCorrector.Update(
                magHeading, ahrsYaw, gyroMag,
                magCalibrator.IsCalibrated, magIsReliable,
                gyroBiasEstimator.IsSettling, handLabel, accelDev);

            // Lock/unlock MagCalibrator based on corrector state
            var snapState = magYawCorrector.CurrentState;
            if (snapState == MagSnapshotYawCorrector.State.Moving ||
                snapState == MagSnapshotYawCorrector.State.CollectingPost ||
                snapState == MagSnapshotYawCorrector.State.Correcting)
            {
                magCalibrator.Lock();
            }
            else
            {
                magCalibrator.Unlock();
            }

            // Accumulate correction (do NOT write back to AHRS — prevents fight-back)
            if (Mathf.Abs(yawCorrection) > 0.001f)
            {
                yawCorrectionAccum += yawCorrection;
            }

            if (imuDiagCounter % 200 == 0)
            {
                Debug.Log($"[IMU-{handLabel}] ahrsY={ahrsYaw:F1} magH={magHeading:F1} " +
                          $"state={magYawCorrector.CurrentState} " +
                          $"drift={magYawCorrector.DetectedDrift:F1} " +
                          $"remaining={magYawCorrector.RemainingCorrection:F1} " +
                          $"yawAccum={yawCorrectionAccum:F1} " +
                          $"gyro={gyroMag:F1}");
            }
        }
        else if (imuDiagCounter % 1000 == 1)
        {
            Debug.LogWarning($"[IMU-{handLabel0}] Stage 3 SKIPPED: " +
                $"magPresent={magCalibrator.IsMagnetometerPresent}, " +
                $"magCalibrated={magCalibrator.IsCalibrated}");
        }

        // ====================================================================
        // Update roll/pitch/yaw from AHRS quaternion
        // ====================================================================
        roll  = Mathf.Atan2(2f * (q0 * q1 + q2 * q3), 1f - 2f * (q1 * q1 + q2 * q2)) * Mathf.Rad2Deg;
        pitch = Mathf.Asin(Mathf.Clamp(-2f * (q1 * q3 - q0 * q2), -1f, 1f)) * Mathf.Rad2Deg;
        yaw   = Mathf.Atan2(2f * (q0 * q3 + q1 * q2), 1f - 2f * (q2 * q2 + q3 * q3)) * Mathf.Rad2Deg;

        // ====================================================================
        // Map to Unity coordinate system + apply finger curl offset
        // ====================================================================
        float fingerAVG = ((index.Item1 + index.Item2) / 2 +
            (middle.Item1 + middle.Item2) / 2 +
            (ring.Item1 + ring.Item2) / 2 +
            (pinky.Item1 + pinky.Item2) / 2) / 4;
        fingerAVG = 45f * fingerAVG / 90f;

        // Apply accumulated yaw correction in earth-frame BEFORE coordinate mapping.
        // Uses PRE-multiply (q_yaw × q_ahrs) to rotate around earth Z-axis.
        // This guarantees zero pitch/roll leakage regardless of hand tilt.
        if (Mathf.Abs(yawCorrectionAccum) > 0.01f)
        {
            float halfYaw = yawCorrectionAccum * Mathf.Deg2Rad * 0.5f;
            float sy = Mathf.Sin(halfYaw);
            float cy = Mathf.Cos(halfYaw);

            float cq0 = cy * q0 - sy * q3;
            float cq1 = cy * q1 - sy * q2;
            float cq2 = cy * q2 + sy * q1;
            float cq3 = sy * q0 + cy * q3;

            q0 = cq0; q1 = cq1; q2 = cq2; q3 = cq3;
        }

        if (!isLeft)
        {
            quaternions = new Quaternion(-q2, q1, q3, q0) * Quaternion.Euler(0, fingerAVG - 45f, 0);
            euler = quaternions.eulerAngles;
        }
        else
        {
            quaternions = new Quaternion(q2, -q1, q3, q0) * Quaternion.Euler(0, fingerAVG - 45f, 0);
            euler = quaternions.eulerAngles;
        }
    }

    /// <summary>
    /// Update accelerometer data.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateAccelerometerValues(byte[] serialBuffer)
    {
        int gap = 0;                                    // used to calculate the gap between the accelerometer data array and the serial buffer array positions.
        uint[] accelerometerData = new uint[6];

        // accelerometer data is located in the positions from 23 to 28 in the serial buffer array.
        for (int i = 23; i <= 28; i++)
        {
            accelerometerData[gap] = serialBuffer[i];
            gap++;
        }

        short x = (short)(((accelerometerData[1]) << 8) + accelerometerData[0]);
        short y = (short)(((accelerometerData[3]) << 8) + accelerometerData[2]);
        short z = (short)(((accelerometerData[5]) << 8) + accelerometerData[4]);

        // Store raw int16 values for the IMU pipeline
        accelerometerRawInt16 = new Vector3(x, y, z);

        // convert data and build 3D vector.
        accelerometer = new Vector3(unchecked(x * 4.0f / 32768.0f),
                                          unchecked(y * 4.0f / 32768.0f),
                                          unchecked(z * 4.0f / 32768.0f));
    }
    /// <summary>
    /// Update gyroscope data.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateGyroscopeValues(byte[] serialBuffer)
    {
        int gap = 0;                                    // used to calculate the gap between the accelerometer data array and the serial buffer array positions.
        uint[] gyroscopeData = new uint[6];             // temporal array for accelerometer data - used for data conversion.

        // gyroscope or angular rate data is located in the positions from 35 to 40 in the serial buffer array.
        for (int i = 35; i <= 40; i++)
        {
            gyroscopeData[gap] = serialBuffer[i];
            gap++;
        }
        short x = (short)(((gyroscopeData[1]) << 8) + gyroscopeData[0]);
        short y = (short)(((gyroscopeData[3]) << 8) + gyroscopeData[2]);
        short z = (short)(((gyroscopeData[5]) << 8) + gyroscopeData[4]);

        // Store raw int16 values for the IMU pipeline
        gyroscopeRawInt16 = new Vector3(x, y, z);

        // convert data and build 3D vector (also used for CalibrateGyro and other consumers)
        if (gyroCalibrated)
        {
            gyroscope = new Vector3(unchecked(x * Mathf.Deg2Rad * 2000.0f / 32768.0f),
                                          unchecked(y * Mathf.Deg2Rad * 2000.0f / 32768.0f),
                                          unchecked(z * Mathf.Deg2Rad * 2000.0f / 32768.0f)) - gyroscopeOffsetValues;
        }
        else
        {
            gyroscope = new Vector3(unchecked(x * Mathf.Deg2Rad * 2000.0f / 32768.0f),
                                          unchecked(y * Mathf.Deg2Rad * 2000.0f / 32768.0f),
                                          unchecked(z * Mathf.Deg2Rad * 2000.0f / 32768.0f));
        }
    }

    /// <summary>
    /// Update magnetometer data.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdateMagnetometerValues(byte[] serialBuffer)
    {
        int gap = 0;                                 // used to calculate the gap between the accelerometer data array and the serial buffer array positions.
        uint[] magnetometerData = new uint[6];       // temporal array for accelerometer data - used for data conversion.

        // magnetometer data is located in the positions from 29 to 34 in the serial buffer array.
        for (int i = 29; i <= 34; i++)
        {
            magnetometerData[gap] = serialBuffer[i];
            gap++;
        }
        short x = (short)(((magnetometerData[0]) << 8) + magnetometerData[1]);
        short y = (short)(((magnetometerData[2]) << 8) + magnetometerData[3]);
        short z = (short)(((magnetometerData[4]) << 8) + magnetometerData[5]);

        // convert data and build 3D vector.
        // MLX90393 sensitivity: XY=0.376 µT/LSB, Z=0.605 µT/LSB
        magnetometer = new Vector3(unchecked(x * 0.376f),
                                          unchecked(y * 0.376f),
                                          unchecked(z * 0.605f));

        // Feed into MagCalibrator for hard-iron correction + reliability detection
        magCorrected = magCalibrator.Update(magnetometer, out magIsReliable);


        if (!magCalibrated)
        {
            ReplaceWithMax(magMaximum, magnetometer);
            ReplaceWithMin(magMinimum, magnetometer);
            magOffset[0] = (magMinimum[0] + magMaximum[0]) / 2;
            magOffset[1] = (magMinimum[1] + magMaximum[1]) / 2;
            magOffset[2] = (magMinimum[0] + magMaximum[2]) / 2;

            magScale[0] = (magMaximum[0] - magMinimum[0]) / 2;
            magScale[1] = (magMaximum[1] - magMinimum[1]) / 2;
            magScale[2] = (magMaximum[2] - magMinimum[2]) / 2;
        }
        magCalibration[0] = (magnetometer.x - magOffset[0]);
        magCalibration[1] = (magnetometer.y - magOffset[1]);
        magCalibration[2] = (magnetometer.z - magOffset[2]);
    }

    /// <summary>
    /// Updates magnometer vaues to be
    /// ready for calibration.
    /// </summary>
    public void UploadMagValues()
    {
        if (isLeft)
        {
            stream.SendCalibratedMagOffsetLeft(new Vector3(magOffset[0], magOffset[1], magOffset[2]));
        }
        else
        {
            stream.SendCalibratedMagOffsetRight(new Vector3(magOffset[0], magOffset[1], magOffset[2]));
        }
    }

    /// <summary>
    /// Calibrates controller gyroscope or resets values 
    /// if it has been already calibrated for
    /// either left or right controller.
    /// </summary>
    public void CalibrateGyro()
    {
        if (!gyroCalibrationDone || !enable)
        {
            return;
        }
        else if (gyroCalibrated)
        {
            gyroscopeOffsetValues = Vector3.zero;
            samplesTaken = 0;
            gyroCalibrated = false;
        }
        if (samplesTaken >= gyroCalibratingSamples)
        {
            gyroscopeOffsetValues /= samplesTaken;
            gyroCalibrationDone = false;
            gyroCalibrated = true;
            stream.DisableDataStreaming();
            if (isLeft)
            {
                stream.SendCalibratedGyroOffsetLeft(gyroscopeOffsetValues);
            }
            else
            {
                stream.SendCalibratedGyroOffsetRight(gyroscopeOffsetValues);
            }

            return;
        }
        Debug.Log("Calibrating Gyroscope");
        gyroscopeOffsetValues += gyroscope;
        samplesTaken++;

    }
    /// <summary>
    /// Calibrates controller magnometer or resets values 
    /// if it has been already calibrated for
    /// either left or right controller.
    /// </summary>
    public void CalibrateMag()
    {
        if (!gyroCalibrationDone || !enable)
        {
            return;
        }
        else if (magCalibrated)
        {
            magOffsetValues = Vector3.zero;
            samplesTaken = 0;
            magCalibrated = false;
        }
        if (samplesTaken >= magCalibratingSamples)
        {
            magOffsetValues /= samplesTaken;
            gyroCalibrationDone = false;
            magCalibrated = true;
            stream.DisableDataStreaming();
            if (isLeft)
            {
                stream.SendCalibratedMagOffsetLeft(magOffsetValues);
            }
            else
            {
                stream.SendCalibratedMagOffsetRight(magOffsetValues);
            }

            return;
        }
        Debug.Log("Calibrating Magnetometer");
        magOffsetValues += magnetometer;
        samplesTaken++;

    }

    /// <summary>
    /// Replaces input Vector3 values to
    /// the set max value.
    /// </summary>
    /// <param name="max">float array - max value for calibration</param>
    /// <param name="values">Vector3 - values that need replacing</param>
    void ReplaceWithMax(float[] max, Vector3 values)
    {
        if (values.x > max[0])
        {
            max[0] = values.x;
        }
        if (values.y > max[1])
        {
            max[1] = values.y;
        }
        if (values.z > max[2])
        {
            max[2] = values.z;
        }
    }

    /// <summary>
    /// Replaces input Vector3 values to
    /// the set min value.
    /// </summary>
    /// <param name="min">float array - min value for calibration</param>
    /// <param name="values">Vector3 - values that need replacing</param>
    void ReplaceWithMin(float[] min, Vector3 values)
    {
        if (values.x < min[0])
        {
            min[0] = values.x;
        }
        if (values.y < min[1])
        {
            min[1] = values.y;
        }
        if (values.z < min[2])
        {
            min[2] = values.z;
        }

    }

    // ==================================== Gesture ====================================

    /// <summary>
    /// Get squeeze gesture data.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>bool</returns>
    private void UpdateSqueezeGesture(byte[] serialBuffer)
    {
        gripTouch = IsBitSet(serialBuffer[10], 0);
        gripClick = IsBitSet(serialBuffer[11], 0);

        float pullValue = SelectBits(serialBuffer[10], 7, 1);
        float forceValue = SelectBits(serialBuffer[14], 7, 1);
        gripPressures = new Tuple<float, float>(pullValue, forceValue);

        squeeze = IsBitSet(serialBuffer[41], 0);
    }

    /// <summary>
    /// Get point independent gesture
    /// status value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdatePointIndependentGesture(byte[] serialBuffer)
    {
        pointIndependent = IsBitSet(serialBuffer[13], 0);
    }

    /// <summary>
    /// Get point B gesture
    /// status value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    private void UpdatePointExcludeTrackpadGesture(byte[] serialBuffer)
    {
        pointExcludeTrackpad = IsBitSet(serialBuffer[14], 0);
    }

    /// <summary>
    /// Get pinch trackpad gesture
    /// status value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    public void UpdatePinchTrackpadGesture(byte[] serialBuffer)
    {
        pinchTrackpad = IsBitSet(serialBuffer[15], 0);
        pinchTrackpadAnalog = SelectBits(serialBuffer[15], 7, 1);
    }

    /// <summary>
    /// Get pinch thumbfinger gesture
    /// status value.
    /// </summary>
    /// <param name="serialBuffer">byte array - array data from the device</param>
    /// <returns>void</returns>
    public void UpdatePinchThumbFingerGesture(byte[] serialBuffer)
    {
        pinchThumbFinger = IsBitSet(serialBuffer[16], 0);
        pinchThumbFingerAnalog = SelectBits(serialBuffer[16], 7, 1);
    }

    /// <summary>
    /// Update the devices charge state.
    /// </summary>
    /// <param name="serialBuffer"></param>
    private void UpdateCharging(byte[] serialBuffer)
    {
        chargingInProgress = IsBitSet(serialBuffer[11], 4);
        if (chargingInProgress)
        {
            chargingComplete = IsBitSet(serialBuffer[12], 0);
        }
    }


    /// <summary>
    /// Get the bit position inside a byte.
    /// Used here to get gesture and
    /// info values.
    /// </summary>
    /// <param name="b">byte - byte where we look for the bit</param>
    /// <param name="pos">int - which position of the byte we want to get</param>
    /// <returns>bool</returns>
    public bool IsBitSet(byte b, int pos)
    {
        return (b & (1 << pos)) != 0;
    }

    /// <summary>
    /// Extracts subsection of bits from a byte
    /// </summary>
    /// <param name="inputByte">The input byte</param>
    /// <param name="bitQuant">How many bits are needed</param>
    /// <param name="startBit">Where to start taking bits from (starts from lsb)</param>
    /// <returns>Value of selection as an int</returns>
    public float SelectBits(byte inputByte, int bitQuant, int startBit)
    {
        int selectedBits = ((1 << bitQuant) - 1) & (inputByte >> (startBit));
        return selectedBits;
    }


    /// <summary>
    /// Resets all attributes initial values back to 0 or False.
    /// </summary>
    /// <returns>void</returns>
    public void ResetValues()
    {
        // Reset fingers
        thumb = new Tuple<float, float>(0f, 0f);
        index = new Tuple<float, float>(0f, 0f);
        middle = new Tuple<float, float>(0f, 0f);
        ring = new Tuple<float, float>(0f, 0f);
        pinky = new Tuple<float, float>(0f, 0f);
        fingerPullData = new float[5] { 0f, 0f, 0f, 0f, 0f };
        fingerForceData = new float[5] { 0f, 0f, 0f, 0f, 0f };
        fingerClickedData = new bool[5] { false, false, false, false, false };
        fingerTouchedData = new bool[5] { false, false, false, false, false };

        // Reset slider
        sliderButton = false;
        sliderValue = 0f;

        // Reset trackpad
        tap = false;
        trackpadCoordinates = new Vector2(0f, 0f);
        trackpadPressures = new Tuple<float, float>(0f, 0f);
        trackpadTouched = false;
        trackpadClicked = false;

        // Reset gestures
        gripTouch = false;
        gripClick = false;
        gripPressures = new Tuple<float, float>(0f, 0f);
        squeeze = false;

        pointIndependent = false;
        pointIndependentAnalog = 0f;

        pointExcludeTrackpad = false;
        pointExcludeTrackpadAnalog = 0f;

        pinchTrackpad = false;
        pinchTrackpadAnalog = 0f;

        pinchThumbFinger = false;
        pinchThumbFingerAnalog = 0f;

        // Reset Tracker
        proxClicked = false;
        proxValue = 0f;
        proxTouched = false;


        // Reset IMU or quaternions
        accelerometer = new Vector3(0f, 0f, 0f);
        gyroscope = new Vector3(0f, 0f, 0f);
        magnetometer = new Vector3(0f, 0f, 0f);
        quaternions = new Quaternion(0f, 0f, 0f, 0f);
        yawCorrectionAccum = 0f;
        // Reset MagSnapshot baseline so it re-captures after orientation reset
        if (magYawCorrector != null)
            magYawCorrector.ResetBaseline();
        // Re-initialize AHRS gravity from next accel reading
        ahrsNeedsGravityInit = true;

        // Reset others
        battery = 0f;
    }
}
