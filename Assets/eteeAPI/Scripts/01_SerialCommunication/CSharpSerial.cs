using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;
using Microsoft.Win32;
using System.Text.RegularExpressions;
using System.IO;
using System.Linq;
using UnityEngine.Analytics;

/// <summary>
/// Retrieves device and port statuses. Also initialises 
/// device calibration and data streaming commands.
/// </summary>
public class CSharpSerial : MonoBehaviour {
    Thread thread;                                              // Separate thread used to read bytes data from the dongle.
    SerialPort stream;                                          // Stream object for connecting the UI to the dongle. This connection is being handle in a different thread.

    [Header("etee Devices")]
    public eteeDevice leftDevice;                               // etee API left device.
    public eteeDevice rightDevice;                              // etee API right device.

    [Header("Serial communication")]
    public string serialPort;                                   // Serial port name used to connect to the dongle.
    public int baudRate;                                        // Baud rate number used to establish the connection to the dongle.
    public int bufferSize;                                      // Max amount of bytes to stack in the buffer.

    public bool looping = true;                                 // Flag to control the loop for the read thread.                              
    public int disconnectedThreshold = 10;                      // Threshold to check wheter there is no data coming from 1 hand.
    private bool streamingData = false;

    private int count = 0;                                      // Bytes internal counter.
    private Queue leftQueue;                                    // Left device queue to stack the data read.
    private Queue rightQueue;                                   // Right device queue to stack the data read.

    public Queue sendCommandQueueLeft;                          // Left device queue to stack the commands to send to the device.
    public Queue sendCommandQueueRight;                         // Right device queue to stack the commands to send to the device.

    [Header("Device connection")]
    public bool leftConnected = false;                          // Flag - true only when the left hand is connected.
    public bool rightConnected = false;                         // Flag - true only when the right hand is connected.
    private bool dongleConnected = false;                       // Flag - true only when the dongle is connected and detected by the UI.

    [Header("Flags")]
    private int rightCounter;                                       // Internal counter to check if no data is coming from the right hand.
    private int leftCounter;                                        // Internal counter to check if no data is coming from the left hand.

    private bool requestingOffsetLeft = true;
    private bool requestingOffsetRight = true;

    private bool sendVibrationToLeft = false;                       // Flag to check whether we send vibration to left hand device.
    private bool sendVibrationToRight = false;                      // Flag to check whether we send vibration to right hand device.
    private bool checkPorts = false;                                // Flag to control whether the dongle ports can be checked by the UI ( because in initialization the port has not serial metods available )
    private int os;                                                 // This variable checks the current user operative system. Then it is used to dinamycally get the dongle port name.

    [Header("Serial commands")]
    // commands to be sent to the device - they are used for program logic purposess (e.g. vibration, calibration).
    private string startCalibrationCommand = "BP+RB";
    private string cancelCalibrationCommand = "BP+CC";
    private string resetOrientationCommand = "BP+RB";
    private string startStreamingData = "BP+AG";
    private string stopStreamingData = "BP+AS";
    private string requestOffsetGyro = "BP+gf";
    private string requestOffsetMag = "BP+mf";

    private string getDongleVersion = "AT+AB";  // TODO: Create function to retrieve version
    private string getEteeVersion = "BP+AB";  // TODO: Create function to retrieve version

    private string enableHaptic = "BP+h1";
    private string disableHaptic = "BP+h0";
    private string sendHaptic = "BL+MR=100";

    private string VID = "239A";  //16-bit vendor number (Vendor ID)
    private string PID = "8029";  //16-bit product number (Product ID)

    [Header("IMU Offsets")]
    public Vector3 gyroLeftOffset;
    public Vector3 gyroRightOffset;

    public Vector3 magLeftOffset;
    public Vector3 magRightOffset;


    // Start is called before the first frame update
    void Start()
    {
        Init();
    }

    // Update is called once per frame
    void Update()
    {
        // check if there is data to send to the devices in the queues.
        // change this method by any used in your app logic.
        ReadingQueues();

        // check if vibration queues are overloaded.
        CheckVibrationQueues();

        // check for vibration commands to left hand.
        if (sendVibrationToLeft)
        {
            DequeueVibrationCommand("left");
        }

        // check for vibration commands to right hand.
        if (sendVibrationToRight)
        {
            DequeueVibrationCommand("right");
        }

        // check ports status.
        CheckPorts();

        CheckIfConnected();

        if (!thread.IsAlive)
        {
            StartThread();
            if (!stream.IsOpen || stream == null)
            {

            }
        }
    }

    /// <summary>
    /// This function is called every fixed framerate frame, if the MonoBehaviour is enabled.
    /// </summary>
    /// <returns>void</returns>
    void FixedUpdate()
    {
        // update no device activity counters.
        UpdateDisconnectedCounters();
    }

    /// <summary>
    /// Init class method.
    /// </summary>
    /// <returns>void</returns>
    private void Init()
    {

        // get current user operative system to detect the port where the dongle is connected.
        os = (int)System.Environment.OSVersion.Platform;

        // start separate thread to read data from the dongle.
        StartThread();

        // set internal flags to check when the devices are connected.
        leftConnected = false;
        rightConnected = false;

        // set data counter to initial value.
        count = 0;
    }

    // ==================================== Connection ====================================

    /// <summary>
    /// Check if the devices are connected and if
    /// so, update top bar devces status text.
    /// </summary>
    private void CheckIfConnected()
    {
        if (rightCounter > disconnectedThreshold)
        {
            rightConnected = false;
        }

        if (leftCounter > disconnectedThreshold)
        {
            leftConnected = false;
        }
    }


    /// <summary>
    /// Check if dongle is 
    /// connected.
    /// </summary>
    /// <returns>bool</returns>
    public bool IsDongleConnected()
    {
        return dongleConnected;
    }

    /// <summary>
    /// Check if device is connected
    /// Pass 0 for left device and
    /// 1 for right device.
    /// </summary>
    /// <param name="device">int - device id. Use 0 for left and 1 for right.</param>
    /// <returns>void</returns>
    public bool IsDeviceConnected(int device)
    {

        // check if parameter is correct.
        if (device > 1)
        {
            return false;

        }

        return (device == 0) ? leftConnected : rightConnected;
    }

    /// <summary>
    /// Update flags to check wheter the
    /// devices are connected or disconnected.
    /// </summary>
    /// <returns>void</returns>
    private void UpdateDisconnectedCounters()
    {
        rightCounter++;
        leftCounter++;
    }

    // ==================================== Ports ====================================

    /// <summary>
    /// Check the port based on 
    /// the user operative system.
    /// </summary>
    /// <returns>void</returns>
    private void SetPort()
    {
        int[] winCheckers = { 0, 1, 2, 3 };
        int[] unixCheckers = { 4, 6, 128 };

        if (winCheckers.Contains(os))
        {
            // set port for windows operative systems.
            SerialPortWindows();
        }
        else if (unixCheckers.Contains(os))
        {
            // set port for mac and linux operative systems.
            SerialPortUnix();
        }
    }

    /// <summary>
    /// Check existing ports and check if
    /// the dongle is connected.
    /// </summary>
    /// <returns>void</returns>
    public void CheckPorts()
    {
        if (checkPorts)
        {
            // check if the dongle is connected.
            if (SerialPort.GetPortNames().Length > 0)
            {
                dongleConnected = true;
            }
            else
            {
                dongleConnected = false;
                Debug.Log("dongle not detected");
            }
        }
    }

    /// <summary>
    /// Get active dongle port for
    /// windows systems.
    /// </summary>
    /// <param name="VID">string - 16-bit vendor number ( Vendor ID )</string>
    /// <param name="PID">string - 16-bit product number ( Product ID )</string>
    /// <returns>void</returns>
    private void SerialPortWindows()
    {

        List<string> names = ComPortNames(VID, PID);
        if(names.Count > 0)
        {
            checkPorts = true;

            if (SerialPort.GetPortNames().Length == 0)
            {
                Debug.LogWarning("etee dongle not detected, please connect the dongle.");
                dongleConnected = false;
            }
            else
            {
                dongleConnected = true;
                foreach (string s in SerialPort.GetPortNames())
                {
                    for (int i = 0; i < names.Count; i++)
                    {

                        if (s.Contains(names[i]))
                        {
                            serialPort = s;
                        }
                    }
                }
            }
        }
        else
        {
            dongleConnected = false;
        }
    }

    /// <summary>
    /// Get active dongle port for
    /// unix systems ( max, linux )
    /// </summary>
    /// <returns>void</returns>
    private void SerialPortUnix()
    {

        List<string> serialPorts = new List<string>();

        string[] cu = Directory.GetFiles("/dev/", "cu.SLAB_USBtoUART");

        foreach (string dev in cu)
        {
            if (dev.StartsWith("/dev/cu.SLAB_USBtoUART"))
            {
                serialPorts.Add(dev);
            }
        }

        if (serialPorts.Count == 1)
        {
            serialPort = serialPorts[0];
        }
    }

    /// <summary>
    /// Get list of com ports names
    /// availables for the dongle to use.
    /// </summary>
    /// <param name="VID">string - 16-bit vendor number ( Vendor ID )</string>
    /// <param name="PID">string - 16-bit product number ( Product ID )</string>
    /// <returns>List</returns>
    List<string> ComPortNames(string VID, string PID)
    {
        string pattern = string.Format("^VID_{0}.PID_{1}", VID, PID);
        Regex _rx = new Regex(pattern, RegexOptions.IgnoreCase);

        List<string> comports = new List<string>();

        RegistryKey rk1 = Registry.LocalMachine;
        RegistryKey rk2 = rk1.OpenSubKey("SYSTEM\\CurrentControlSet\\Enum");

        foreach (string s3 in rk2.GetSubKeyNames())
        {
            RegistryKey rk3 = rk2.OpenSubKey(s3);

            foreach (string s in rk3.GetSubKeyNames())
            {

                if (_rx.Match(s).Success)
                {
                    RegistryKey rk4 = rk3.OpenSubKey(s);

                    foreach (string s2 in rk4.GetSubKeyNames())
                    {
                        RegistryKey rk5 = rk4.OpenSubKey(s2);
                        RegistryKey rk6 = rk5.OpenSubKey("Device Parameters");

                        try
                        {
                            string port = (string)rk6.GetValue("PortName");
                            if (port != null)
                                comports.Add(port);
                        }
                        catch { }
                    }
                }
            }
        }

        return comports;
    }

    /// <summary>
    /// Get the bit position inside
    /// a byte.
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

    // ==================================== Haptics ====================================

    /// <summary>
    /// Enables device haptic feedback
    /// </summary>
    public void EnableHaptics()
    {
        SendCommandToDevice(enableHaptic);
    }

    /// <summary>
    /// Disables device haptic feedback
    /// </summary>
    public void DisableHaptics()
    {
        SendCommandToDevice(disableHaptic);
    }

    /// <summary>
    /// Send Vibration Commands to device.
    /// </summary>
    /// <param name="hand">string -  which hand data is being sent</param>
    public void SendVibrationsCommands(string hand)
    {
        string command;
        if (hand.ToLower() == "left")
        {
            command = sendHaptic;
        }
        else
        {
            command = sendHaptic;
        }
        SendCommandToDevice(command);
    }

    /// <summary>
    /// Send vibration command
    /// to the devices.
    /// </summary>
    /// <param name="hand">string - to which hand the vibration command is being sent</param>
    /// <returns>void</returns>
    private void DequeueVibrationCommand(string hand)
    {
        // dequeue for the left hand device.
        if (hand == "left")
        {
            string command1 = (string)sendCommandQueueLeft.Dequeue();

            sendVibrationToLeft = false;
            sendCommandQueueLeft.Clear();

            stream.WriteLine(command1);
        }

        // dequeue for the right hand device.
        if (hand == "right")
        {
            string command2 = (string)sendCommandQueueRight.Dequeue();

            sendVibrationToRight = false;
            sendCommandQueueRight.Clear();

            stream.WriteLine(command2);
        }
    }

    /// <summary>
    /// Release items from vibration queue.
    /// </summary>
    public void CheckVibrationQueues()
    {

        if (!leftConnected && sendCommandQueueLeft.Count > 0)
        {
            sendCommandQueueLeft.Clear();
        }

        if (!rightConnected && sendCommandQueueRight.Count > 0)
        {
            sendCommandQueueRight.Clear();
        }
    }

    // ==================================== Calibration ====================================
    /// <summary>
    /// Send start calibration command
    /// to the device.
    /// </summary>
    /// <returns>void</returns>
    public void SendStartCalibrationCommand()
    {
        SendCommandToDevice(startCalibrationCommand);
    }

    /// <summary>
    /// Send cancel calibration command
    /// to the device.
    /// </summary>
    /// <returns>void</returns>
    public void SendCancelCalibrationCommand()
    {
        SendCommandToDevice(cancelCalibrationCommand);
    }

    /// <summary>
    /// Send reset orientation during calibration command
    /// to the device.
    /// </summary>
    /// <returns>void</returns>
    public void SendResetOrientationCalibrationCommand()
    {
        SendCommandToDevice(resetOrientationCommand);
    }

    /// <summary>
    /// Calibrates Gyroscope offset 
    /// for left controller
    /// </summary>
    /// <param name="offset">gyroscope offset values </param>
    /// <returns></returns>
    private IEnumerator SendCalibratedGyroOffsetLeftCoroutine(Vector3 offset)
    {

        yield return new WaitForSeconds(0.3f);
        string message = "BL+gf=a" + offset.x.ToString() + "," + offset.y.ToString() + "," + offset.z.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(0.9f);
        EnableDataStreaming();
    }

    /// <summary>
    /// Begins the calibration of the 
    /// gyroscope for left controller
    /// </summary>
    /// <param name="offset">gyroscope offset values</param>
    public void SendCalibratedGyroOffsetLeft(Vector3 offset)
    {
        StartCoroutine(SendCalibratedGyroOffsetLeftCoroutine(offset));

    }

    /// <summary>
    /// Calibrates Gyroscope offset 
    /// for right controller
    /// </summary>
    /// <param name="offset">gyroscope offset values </param>
    /// <returns></returns>
    private IEnumerator SendCalibratedGyroOffsetRightCoroutine(Vector3 offset)
    {
        yield return new WaitForSeconds(0.5f);
        string message = "BR+gf=a" + offset.x.ToString() + "," + offset.y.ToString() + "," + offset.z.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(0.6f);
    }

    /// <summary>
    /// Begins the calibration of the 
    /// gyroscope for right controller
    /// </summary>
    /// <param name="offset">gyroscope offset values</param>
    public void SendCalibratedGyroOffsetRight(Vector3 offset)
    {
        StartCoroutine(SendCalibratedGyroOffsetRightCoroutine(offset));

    }

    /// <summary>
    /// Calibrates Magnetometer offset 
    /// for left controller
    /// </summary>
    /// <param name="offset">Magnetometer offset values </param>
    /// <returns></returns>
    private IEnumerator SendCalibratedMagOffsetLeftCoroutine(Vector3 offset)
    {
        yield return new WaitForSeconds(1f);
        string message = "BL+mf=X" + offset.x.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);

        message = "BL+mf=Y" + offset.y.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);

        message = "BL+mf=Z" + offset.z.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
    }

    /// <summary>
    /// Begins the calibration of the 
    /// magnetometer for left controller
    /// </summary>
    /// <param name="offset">Magnetometer offset values</param>
    public void SendCalibratedMagOffsetLeft(Vector3 offset)
    {
        StartCoroutine(SendCalibratedMagOffsetLeftCoroutine(offset));

    }

    /// <summary>
    /// Calibrates Magnetometer offset 
    /// for right controller
    /// </summary>
    /// <param name="offset">Magnetometer offset values </param>
    /// <returns></returns>
    private IEnumerator SendCalibratedMagOffsetRightCoroutine(Vector3 offset)
    {
        yield return new WaitForSeconds(2f);
        string message = "BR+mf=X" + offset.x.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);

        message = "BR+mf=Y" + offset.y.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);

        message = "BR+mf=Z" + offset.z.ToString();
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
        SendCommandToDevice(message);
        yield return new WaitForSeconds(1f);
    }

    /// <summary>
    /// Begins the calibration of the 
    /// magnetometer for right controller
    /// </summary>
    /// <param name="offset">Magnetometer offset values</param>
    public void SendCalibratedMagOffsetRight(Vector3 offset)
    {
        StartCoroutine(SendCalibratedMagOffsetRightCoroutine(offset));

    }

    // ==================================== Streaming ====================================
    /// <summary>
    /// Send command to the device.
    /// </summary>
    /// <param name="command">string -  command to be sent</param>
    /// <returns>void</returns>
    private void SendCommandToDevice(string command)
    {
        if (stream == null)
        {
            return;
        }
        if (stream.IsOpen)
        {
            if (command == startStreamingData)
            {
                streamingData = true;
            }
            Debug.Log("Command sent to devices: " + command);
            stream.WriteLine(command);
            stream.BaseStream.Flush();
        }
    }


    /// <summary>
    /// Separate thread to read data from the
    /// dongle and sent to the queues.
    /// Then the system reads from the queues
    /// and triggers the logic.
    /// </summary>
    /// <returns>void</returns>
    private void StartThread()
    {
        // queues to read data from the device.
        leftQueue = Queue.Synchronized(new Queue());
        rightQueue = Queue.Synchronized(new Queue());

        // queues to send commands to the devices.
        sendCommandQueueLeft = Queue.Synchronized(new Queue());
        sendCommandQueueRight = Queue.Synchronized(new Queue());

        // start separate thread to read the data from the dongle - ThreadLoop is the method passed to the new thread.
        thread = new Thread(ThreadLoop);
        thread.Start();
    }

    /// <summary>
    /// Enable Data Streaming through dongle
    /// </summary>
    /// <returns>void</returns>
    public void EnableDataStreaming()
    {
        SendCommandToDevice(startStreamingData);
    }

    /// <summary>
    /// Disable Data Streaming through dongle
    /// </summary>
    /// <returns>void</returns>
    public void DisableDataStreaming()
    {
        SendCommandToDevice(stopStreamingData);
    }


    /// <summary>
    /// Reading queues.
    /// Read byte data from the queue and
    /// send data to the hand controllers.
    /// </summary>
    /// <returns>void</returns>
    private void ReadingQueues()
    {
        byte[] data;
        // read enqueue data for the right device.
        if (rightQueue.Count > 0)
        {
            // update right device status.
            rightConnected = true;
            // build the data and send to the righr hand class.
            for (int i = 0; i < rightQueue.Count; i++)
            {
                data = (byte[])rightQueue.Dequeue();
                // etee api device
                rightDevice.UpdateValuesFromController(data);
            }

            // update right counter to detect wheter the right device is sleeping.
            rightCounter = 0;
        }

        // read enqueue data for the left device.
        if (leftQueue.Count > 0)
        {

            // update left device status.
            leftConnected = true;
            // build the data and send to the right hand class.
            for (int i = 0; i < leftQueue.Count; i++)
            {
                data = (byte[])leftQueue.Dequeue();
                // etee api device.
                leftDevice.UpdateValuesFromController(data);
            }

            // update left counter to detect wheter the left device is sleeping.
            leftCounter = 0;
        }
    }

    /// <summary>
    /// Read bytes data from the dongle
    /// and send to the queues for each device.
    /// This method is pemanently running
    /// in the separate thread.
    /// </summary>
    /// <returns>void</returns>
    private void ThreadLoop()
    {
        dongleConnected = false;
        checkPorts = false;
        serialPort = "";
        // set the port.
        while (!dongleConnected)
        {
            SetPort();
        }

        // set arrays for reading and storing byte data from the devices.
        byte[] serialData = new byte[9999];
        byte[] lastByte = new byte[1];

        // set extra data variables.
        int bufferLimit = 0xFF;
        bool is_right;

        // ensure the counter is initialized to 0.
        count = 0;

        // open the stream reading from the port.
        stream = new SerialPort(serialPort, baudRate);
        stream.Encoding = System.Text.Encoding.UTF8;
        stream.Parity = Parity.None;
        stream.DataBits = 8;
        stream.StopBits = StopBits.One;
        stream.DtrEnable = true;
        stream.Open();
        int counter = 0;
        bool requestGyr = true;
        bool requestMag = false;    // Magnetometer value only needed when using Absolute Orientation (default set to Relative Orientation - accelerometer and gyroscope -)
        requestingOffsetLeft = true;
        requestingOffsetRight = true;
        // separate thread loop to continuosly read data from the device.
        while (looping)
        {
            if (!dongleConnected)
            {
                break;
            }

            if (!streamingData && !requestingOffsetLeft && !requestingOffsetRight)
            {
                EnableDataStreaming();
            }

            if (requestingOffsetLeft || requestingOffsetRight)
            {
                while (requestGyr)
                {
                    SendCommandToDevice(requestOffsetGyro);
                    counter++;
                    string incData = stream.ReadLine();
                    if (incData.Contains("R:gf="))
                    {
                        string[] split = incData.Split(':', ' ');
                        gyroRightOffset = new Vector3(float.Parse(split[2]), float.Parse(split[4]), float.Parse(split[6]));

                        rightDevice.gyroscopeOffsetValues = gyroRightOffset;
                        rightDevice.gyroCalibrated = true;
                    }
                    else if (incData.Contains("L:gf="))
                    {
                        string[] split = incData.Split(':', ' ');
                        gyroLeftOffset = new Vector3(float.Parse(split[2]), float.Parse(split[4]), float.Parse(split[6]));
                        leftDevice.gyroscopeOffsetValues = gyroLeftOffset;
                        leftDevice.gyroCalibrated = true;
                    }
                    if (leftDevice.gyroCalibrated && rightDevice.gyroCalibrated)
                    {
                        requestGyr = false;
                    }
                    Thread.Sleep(100);
                }
                while (requestMag)
                {
                    SendCommandToDevice(requestOffsetMag);                   
                    string incData = stream.ReadLine();
                    if (incData.Contains("R:mf="))
                    {
                        string[] split = incData.Split(':', ' ');
                        magRightOffset = new Vector3(float.Parse(split[2]), float.Parse(split[4]), float.Parse(split[6]));
                        if (magRightOffset.x != 0.0)
                        {
                            rightDevice.magOffset[0] = magRightOffset.x;
                            rightDevice.magOffset[1] = magRightOffset.y;
                            rightDevice.magOffset[2] = magRightOffset.z;
                        }
                        rightDevice.magOffsetValues = magRightOffset;
                        rightDevice.magCalibrated = true;

                    }
                    else if (incData.Contains("L:mf="))
                    {
                        string[] split = incData.Split(':', ' ');
                        magLeftOffset = new Vector3(float.Parse(split[2]), float.Parse(split[4]), float.Parse(split[6]));
                        if (magLeftOffset.x != 0.0)
                        {
                            leftDevice.magOffset[0] = magLeftOffset.x;
                            leftDevice.magOffset[1] = magLeftOffset.y;
                            leftDevice.magOffset[2] = magLeftOffset.z;
                        }
                        leftDevice.magOffsetValues = magLeftOffset;
                        leftDevice.magCalibrated = true;
                    }

                    if (leftDevice.magCalibrated && rightDevice.magCalibrated)
                    {
                        requestMag = false;
                    }
                }
                if (!requestGyr && !requestMag)
                {
                    requestingOffsetLeft = false;
                    requestingOffsetRight = false;
                    Debug.Log("Gyroscope and Magnetometer offsets set");
                }

            }
            while (stream.BytesToRead > 0 && !requestingOffsetLeft && !requestingOffsetRight)
            {
                // read data from the dongle.
                // get the last byte and save into the data stack.
                stream.Read(lastByte, 0, 1);
                serialData[count] = lastByte[0];

                // The end of each package is set when the device sends
                // two 255 in a row. If that happens, the limit
                // of the package has been reached.
                if (count > 0 && serialData[count] == bufferLimit && serialData[count - 1] == bufferLimit)
                {
                    if (count == bufferSize)
                    {
                        streamingData = true;

                        // The 17 position in the bytes contains the
                        // bits for general information. To detect from
                        // which device the data comes from, we check
                        // if the first bit is 0 or 1. 0 is for data
                        // coming from the left device and 1 is data
                        // coming from the right device.
                        is_right = IsBitSet(serialData[11], 3);

                        // enqueue device data.
                        if (is_right)
                        {
                            rightQueue.Enqueue(serialData);
                        }
                        else
                        {
                            leftQueue.Enqueue(serialData);
                        }

                        // reset data stack to receive more data packages.
                        serialData = new byte[9999];
                        count = 0;
                        stream.BaseStream.Flush();

                    }
                    else
                    {

                        // reset data stack to receive more data packages.
                        serialData = new byte[9999];
                        count = 0;
                        stream.BaseStream.Flush();
                    }
                }
                else
                {
                    count++;
                }
            }
            stream.BaseStream.Flush();
        }

        dongleConnected = false;

        // stop the thread if the while statement is broken and close connection to the port.
        StopThread();
        thread.Abort();

        if (stream.IsOpen)
        {
            stream.Close();
        }
    }

    /// <summary>
    /// Stop the spearate thread.
    /// </summary>
    /// <returns>void</returns>
    public void StopThread()
    {
        lock (this)
        {
            looping = false;
        }
        streamingData = false;
    }



    /// <summary>
    /// Event listener triggered
    /// when the application stops
    /// running.
    /// It stops the separate threads
    /// and close the connection to 
    /// the serial port.
    /// </summary>
    /// <returns>void</returns>
    public void OnApplicationQuit()
    {
        DisableDataStreaming();
        StopThread();

        if (stream.IsOpen)
        {
            stream.Close();
        }
    }


}
