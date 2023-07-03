using System;
using UnityEngine;

/// <summary>
/// Retrieves values from the API,
/// through Get() commands.
/// </summary>
public class eteeAPI : MonoBehaviour {
    public static eteeAPI instance;                                 // Static instance to make this API available in the whole application scope.
    public CSharpSerial serialRead;                                 // Serial reader class component reference.
    public eteeDevice leftDevice;                                   // etee left device from where the data is retrieved class component refernece.
    public eteeDevice rightDevice;                                  // etee right device from where the data is retrieved class componer reference.

    /// <summary>
    /// Awake is called when the script instance is being loaded.
    /// </summary>
    /// <returns>void</returns>
    void Awake()
    {
        if (instance == null)
        {
            instance = this;
        }
    }

    /// <summary>
    /// Reset the controller parameters to 0
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    public void ResetControllerValues(int device)
    {
        Debug.Log("Resetting controller values in API");
        if (device == 0)
        {
            leftDevice.ResetValues();
        }

        else if (device == 1)
        {
            rightDevice.ResetValues();
        }
    }

    /// <summary>
    /// Restart data streaming on controllers
    /// </summary>
    public void RestartStreaming()
    {
        serialRead.DisableDataStreaming();
        serialRead.EnableDataStreaming();
    }

    // ==================================== Status ====================================

    /// <summary>
    /// Checks if the dongle is
    /// connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsDongleDeviceConnected()
    {
        return serialRead.IsDongleConnected();
    }

    /// <summary>
    /// Disconnects the system.
    /// Stops serial read reading
    /// data thread.
    /// </summary>
    /// <returns>void</returns>
    public void Disconnect()
    {
        serialRead.StopThread();
    }

    /// <summary>
    /// Checks if both controllers 
    /// are connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsBothDevicesConnected()
    {
        if(serialRead.IsDeviceConnected(0) && serialRead.IsDeviceConnected(1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>
    /// Checks if either controller 
    /// is connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsAnyDeviceConnected()
    {
        if (serialRead.IsDeviceConnected(0) || serialRead.IsDeviceConnected(1))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    /// <summary>
    /// Checks if left device
    /// is connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsLeftDeviceConnected()
    {
        // 0 is used for left device as standard in all the etee api library.
        return serialRead.IsDeviceConnected(0);
    }

    /// <summary>
    /// Check if right device
    /// is connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsRightDeviceConnected()
    {
        // 1 us used for right device as standard in all the etee api library.
        return serialRead.IsDeviceConnected(1);
    }

    /// <summary>
    /// Get port name used
    /// to establish the connection.
    /// </summary>
    /// <returns>string</returns>
    public string GetPortName()
    {
        return serialRead.serialPort;
    }

    /// <summary>
    /// Get battery value.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>float</returns>
    public float GetBattery(int device)
    {

        // check the parameter is correct.
        if (device > 1)
        {
            return 0f;
        }

        return (device == 0) ? leftDevice.battery : rightDevice.battery;
    }

    /// <summary>
    /// Wrapper method to check to
    /// which port the dongle is
    /// connected. You need to pass
    /// the port name as a parameter.
    /// </summary>
    /// <param name="portName">string - name of the port to check</param>
    public bool CheckPort(string portName)
    {
        if (GetPortName() == portName)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Wrapper method for checking
    /// if the device requested is
    /// the right hand.
    /// </summary>
    /// <param name="device">int - device to from where you get the data from. 0 for left and 1 for right</param>
    public bool IsRightHand(int device)
    {
        if (device == 0)
        {
            return false;
        }
        return true;
    }

    // ==================================== Finger ====================================

    /// <summary>
    /// Get single finger data.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <parma name="fingerIndex">int - finger index. The correlation is the following: 0 - Thumb, 1 - Index, 2 - Middle, 3 - Ring, 4 - Pinky</param>
    /// <returns>float</returns>
    public Tuple<float, float> GetFinger(int device, int fingerIndex)
    {
        Tuple<float, float> value = new Tuple<float, float>(0f, 0f);

        // check that the value requested is valid.
        if (fingerIndex < 0 || fingerIndex > 4)
        {
            return value;
        }

        switch (fingerIndex)
        {
            case 0:
                value = (device == 0) ? leftDevice.thumb : rightDevice.thumb;
                break;
            case 1:
                value = (device == 0) ? leftDevice.index : rightDevice.index;
                break;
            case 2:
                value = (device == 0) ? leftDevice.middle : rightDevice.middle;
                break;
            case 3:
                value = (device == 0) ? leftDevice.ring : rightDevice.ring;
                break;
            case 4:
                value = (device == 0) ? leftDevice.pinky : rightDevice.pinky;
                break;
            default:
                value = new Tuple<float, float>(0f, 0f);
                break;
        }

        return value;
    }

    /// <summary>
    /// Get all fingers data pull pressure data
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>float[]</returns>
    public float[] GetAllFingersPull(int device)
    {

        if (device > 1)
        {
            float[] nullOp = new float[1];
            return nullOp;
        }

        return (device == 0) ? leftDevice.fingerPullData : rightDevice.fingerPullData;
    }

    /// <summary>
    /// Get all fingers data force pressure data
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>float[]</returns>
    public float[] GetAllFingersForce(int device)
    {
        if (device > 1)
        {
            float[] nullOp = new float[1];
            return nullOp;
        }

        return (device == 0) ? leftDevice.fingerForceData : rightDevice.fingerForceData;
    }


    /// <summary>
    /// Starts calibration of fingers.
    /// </summary>
    public void CalibrateFingers()
    {
        serialRead.SendStartCalibrationCommand();
    }

    // ==================================== Trackpad ====================================

    /// <summary>
    /// Get trackpad axis value.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <param name="axis">char - axis whose value you want to retrieve.true Values allowed are 'x' or 'y'</param>
    /// <returns>float</returns>
    public float GetTrackpadPositionSingleAxis(int device, char axis)
    {
        // check that the value requested is valid.
        if (axis != 'x' && axis != 'y')
        {
            return 0f;
        }
        else
        {
            Vector2 data = (device == 0) ? leftDevice.trackpadCoordinates : rightDevice.trackpadCoordinates;
            return (axis == 'x') ? data.x : data.y;
        }
    }

    /// <summary>
    /// Get trackpad axis values.
    /// </sumamry>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>Vector2</returns>
    public Vector2 GetTrackpadPosition(int device)
    {

        // check that parameter is correct.
        if (device > 1)
        {
            return Vector2.zero;
        }

        return (device == 0) ? leftDevice.trackpadCoordinates : rightDevice.trackpadCoordinates;
    }

    /// <summary>
    /// Get trackpad pressure values.
    /// </sumamry>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>Vector2</returns>
    public Tuple<float, float> GetTrackpadPressures(int device)
    {
        // check that parameter is correct.
        if (device > 1 | device < 0)
        {
            return new Tuple<float, float>(0f, 0f);
        }

        return (device == 0) ? leftDevice.trackpadPressures : rightDevice.trackpadPressures;
    }

    /// <summary>
    /// Check if the trackpad
    /// has been tapped.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetTrackpadTapped(int device)
    {

        // check that device parameter is correct.
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.trackpadTapped : rightDevice.trackpadTapped;
    }

    /// <summary>
    /// Check if tap has been
    /// performed by the user.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>void</returns>
    public Tuple<bool, bool> GetTap(int device)
    {

        // check that device parameter is correct.
        if (device > 1 | device < 0)
        {
            return new Tuple<bool, bool>(false, false);
        }

        return (device == 0) ? leftDevice.taps : rightDevice.taps;
    }

    // ==================================== Slider ====================================


    /// <summary>
    /// Wrapper method for retrieving the Y-location value
    /// of the slider.
    /// </summary>
    /// <param name="device">int - device to from where you get the data from. 0 for left and 1 for right</param>
    public float GetSliderPosition(int device)
    {
        if (device > 1)
        {
            return 0f;
        }
        return (device == 0) ? leftDevice.sliderValue : rightDevice.sliderValue;
    }

    /// <summary>
    /// Check if the Slider
    /// button is pressed.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetSliderTouched(int device)
    {

        // check that device parameter is correct.
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.sliderButton : rightDevice.sliderButton;
    }

    /// <summary>
    /// Check if the Slider up or down buttons are pressed.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public Tuple<bool, bool> GetSliderUpDownTouched(int device)
    {
        // check that device parameter is correct.
        if (device > 1)
        {
            return new Tuple<bool, bool>(false, false);
        }

        return (device == 0) ?
            new Tuple<bool, bool>(leftDevice.sliderUpButton, leftDevice.sliderDownButton) :
            new Tuple<bool, bool>(rightDevice.sliderUpButton, rightDevice.sliderDownButton);
    }


    // ==================================== Rotation ====================================

    /// <summary>
    /// Grabs roll, pitch and yaw rotation data from
    /// either left or right hand.
    /// </summary>
    /// <param name="device">int - device from where you get the data from. 0 for left and 1 for right</param>
    /// <returns></returns>
    public Vector3 GetRotations(int device)
    {
        // check that device parameter is correct.
        if (device > 1 | device < 0)
        {
            return new Vector3(0f, 0f, 0f);
        }

        return (device == 0) ? new Vector3(leftDevice.roll, leftDevice.pitch, leftDevice.yaw)
            : new Vector3(rightDevice.roll, rightDevice.pitch, rightDevice.yaw);
    }

    /// <summary>
    /// Wrapper method to get all
    /// the quaternions for rotation.
    /// </summary>
    /// <param name="device">int - device to from where you get the data from. 0 for left and 1 for right</param>
    public float[] GetQuaternionValues(int device)
    {

        // get quaternions data from the get single quaternion method from the API
        char[] quaternionKeys = { 'w', 'x', 'y', 'z' };
        float[] data = new float[quaternionKeys.Length];

        for (int i = 0; i < quaternionKeys.Length; i++)
        {
            data[i] = GetQuaternionComponent(device, quaternionKeys[i]);
        }

        return data;
    }

    /// <summary>
    /// Get single quaternion component
    /// value for rotation.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <param name="component">char - component name. Values allowed are: 'x', 'y', 'z' and 'w'</param>
    /// <returns>float</returns>
    public float GetQuaternionComponent(int device, char component)
    {

        float value = 0f;

        // check that parameter device is correct.
        if (device > 1)
        {
            return 0f;
        }

        Quaternion data = (device == 0) ? leftDevice.quaternions : rightDevice.quaternions;

        switch (component)
        {
            case 'x':
                value = data.x;
                break;
            case 'y':
                value = data.y;
                break;
            case 'z':
                value = data.z;
                break;
            case 'w':
                value = data.w;
                break;
            default:
                value = 0f;
                break;
        }

        return value;
    }

    /// <summary>
    /// Get quaternions data
    /// values for rotation.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>Quaternion</returns>
    public Quaternion GetQuaternions(int device)
    {

        // check that device number is correct.
        if (device > 1)
        {
            return new Quaternion(0f, 0f, 0f, 0f);
        }

        return (device == 0) ? leftDevice.quaternions : rightDevice.quaternions;
    }

    /// <summary>
    /// Get euler data
    /// for velocity.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>Vector3</returns>
    public Vector3 GetEuler(int device)
    {
        // check that device parameter is correct.
        if (device > 1)
        {
            return new Vector3(0f, 0f, 0f);
        }

        return (device == 0) ? leftDevice.euler : rightDevice.euler;
    }


    /// <summary>
    /// Get acceleromenter axis
    /// data for velocity.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <parma name="axis">char - Axis to be retrieved. Allowed values are: 'x', 'y', 'z'</param>
    /// <returns>float</returns>
    public float GetAccelerometerSingleAxis(int device, char axis)
    {
        float value = 0f;

        // check that the device parameter is correct.
        if (device > 1)
        {
            return value;
        }

        Vector3 data = (device == 0) ? leftDevice.accelerometer : rightDevice.accelerometer;

        switch (axis)
        {
            case 'x':
                value = data.x;
                break;
            case 'y':
                value = data.y;
                break;
            case 'z':
                value = data.z;
                break;
            default:
                value = 0f;
                break;
        }

        return value;
    }

    /// <summary>
    /// Wraper method to get all
    /// the accelerometer values from
    /// the device.
    /// </summary>
    /// <param name="device">int - device to from where you get the data from. 0 for left and 1 for right</param>
    public float[] GetAccelerometer(int device)
    {
        char[] keys = { 'x', 'y', 'z' };
        float[] data = new float[keys.Length];

        for (int i = 0; i < keys.Length; i++)
        {
            data[i] = GetAccelerometerSingleAxis(device, keys[i]);
        }

        return data;
    }

    /// <summary>
    /// Get acceleromenter data
    /// for velocity.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>Vector3</returns>
    public Vector3 GetAccel(int device)
    {

        // check that device parameter is correct.
        if (device > 1)
        {
            return new Vector3(0f, 0f, 0f);
        }

        return (device == 0) ? leftDevice.accelerometer : rightDevice.accelerometer;
    }

    public Vector3 GetGyro(int device)
    {
        // check that device parameter is correct.
        if (device > 1)
        {
            return new Vector3(0f, 0f, 0f);
        }

        return (device == 0) ? leftDevice.gyroscope : rightDevice.gyroscope;
    }

    /// <summary>
    /// Flags both controllers gyroscopes
    /// to be calibrated.
    /// </summary>
    public void CalibrateDevicesGyro()
    {
        leftDevice.gyroCalibrationDone = true;
        rightDevice.gyroCalibrationDone = true;
    }

    /// <summary>
    /// Checks if the input device's gyroscope
    /// has been calibrated.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns></returns>
    public bool GetIfDeviceGyroIsCalibrated(int device)
    {
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.gyroCalibrated : rightDevice.gyroCalibrated;
    }

    public Vector3 GetMag(int device)
    {
        // check that device parameter is correct.
        if (device > 1)
        {
            return new Vector3(0f, 0f, 0f);
        }

        return (device == 0) ? leftDevice.magnetometer : rightDevice.magnetometer;
    }

    /// <summary>
    /// Flags both controllers magnometer
    /// to be calibrated.
    /// </summary>
    /// <param name="enable">bool - if the magnometer enabled or not.</param>
    public void CalibrateDevicesMag(bool enable)
    {
        leftDevice.magCalibrated = !enable;
        rightDevice.magCalibrated = !enable;
        if (!enable)
        {
            leftDevice.UploadMagValues();
            rightDevice.UploadMagValues();
        }

    }

    // ==================================== Gestures ====================================

    /// <summary>
    /// Checks if a squeeze
    /// gesture is being performed
    /// by the user.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetIsSqueezeGesture(int device)
    {
        // check that the device number is correct.
        if (device > 1)
        {
            return false;
        }
        return (device == 0) ? leftDevice.squeeze : rightDevice.squeeze;
    }

    /// <summary>
    /// Check if the user
    /// is performing a Point Independent
    /// gesture.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetIsPointIndependentGesture(int device)
    {

        // check if device number is correct.
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.pointIndependent : rightDevice.pointIndependent;
    }

    /// <summary>
    /// Check if the user
    /// is performing a Point 
    /// Exclude Trackpad gesture.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetIsPointExcludeTrackpadGesture(int device)
    {

        // check if device number is correct.
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.pointExcludeTrackpad : rightDevice.pointExcludeTrackpad;
    }

    /// <summary>
    /// Check if the user 
    /// is performing a pinch trackpad
    /// gesture.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetIsPinchTrackpadGesture(int device) 
    {

        // check if device number is correct.
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.pinchTrackpad : rightDevice.pinchTrackpad;
    }

    /// <summary>
    /// Check if the user 
    /// is performing a pinch thumbfinger
    /// gesture.
    /// </summary>
    /// <param name="device">int - device number. Use 0 for left and 1 for right</param>
    /// <returns>bool</returns>
    public bool GetIsPinchThumbFingerGesture(int device)
    {
        
        // check if device number is correct
        if (device > 1)
        {
            return false;
        }

        return (device == 0) ? leftDevice.pinchThumbFinger : rightDevice.pinchThumbFinger;
    }

    // ==================================== Haptics ====================================

    /// <summary>
    /// Enables controller
    /// haptic feedback.
    /// </summary>
    public void EnableHaptics()
    {
        serialRead.EnableHaptics();
    }

    /// <summary>
    /// Disables controller
    /// haptic feedback.
    /// </summary>
    public void DisableHaptics()
    {
        serialRead.DisableHaptics();
    }

}
