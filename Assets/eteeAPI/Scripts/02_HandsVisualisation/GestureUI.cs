using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GestureUI : MonoBehaviour
{
    // gestures.
    public GameObject[] baseGesturesL;                                 // Base gestures from first etee version.
    public GameObject[] baseGesturesR;

    public eteeAPI api;                                             // Etee api to read events from the dongle.

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        ListeningForGestureEvents(0);
        ListeningForGestureEvents(1);
    }

    /// <summary>
    /// Listen for gesture events
    /// coming from the device.
    /// </summary>
    /// <param name="device">int - device ID. Pass the value 0 to get data from the left device and pass value 1 to get data from the right device</param>
    private void ListeningForGestureEvents(int device)
    {
        bool[] events = new bool[7];

        // check grip gesture.
        events[0] = api.GetIsSqueezeGesture(device);

        // check point independent gesture.
        events[1] = api.GetIsPointIndependentGesture(device);

        // check pinch trackpad gesture.
        events[2] = api.GetIsPinchTrackpadGesture(device);

        // check pinch thumbfinger gesture
        events[5] = api.GetIsPinchThumbFingerGesture(device);

        // check point exclude trackpad gesture.
        events[6] = api.GetIsPointExcludeTrackpadGesture(device);

        // send data to left device.
        if (device == 0)
        {

            // base gestures.
            DisplaySqueezeGesture(events[0], baseGesturesL);
            DisplayPointIndependentGesture(events[6], events[0], events[1], baseGesturesL);
            DisplayPointExcludeTrackpadGesture(events[1], events[0], baseGesturesL);
            DisplayPinchTrackpadGesture(events[5], events[0], events[1], events[2], baseGesturesL);
            DisplayPinchThumbFingerGesture(events[2], events[0], events[1], events[5], baseGesturesL);

        }
        else
        {        // send data to the right device.

            // base gestures.
            DisplaySqueezeGesture(events[0], baseGesturesR);
            DisplayPointIndependentGesture(events[6], events[0], events[1], baseGesturesR);
            DisplayPointExcludeTrackpadGesture(events[1], events[0], baseGesturesR);
            DisplayPinchTrackpadGesture(events[5], events[0], events[6], events[2], baseGesturesR);
            DisplayPinchThumbFingerGesture(events[2], events[0], events[6], events[5], baseGesturesR);

        }

    }
    /// <summary>
    /// Display squeeze gesture
    /// in the UI.
    /// </summary>
    /// <param name="isGesture">bool - wheter the gesture is being performed or not</param>
    public void DisplaySqueezeGesture(bool isGesture, GameObject[] baseGestures)
    {
        baseGestures[2].SetActive(isGesture);   
    }

    /// <summary>
    /// Check if a Point Exclude Trackpad gesture is
    /// being performed by the user.
    /// </summary>
    /// <param name="isGesture">bool - wheter the gesture is being performed.</param>
    public void DisplayPointExcludeTrackpadGesture(bool isGesture, bool isSqueeze, GameObject[] baseGestures)
    {
        if (isSqueeze)
        {
            baseGestures[4].SetActive(false);
        }
        else
        {
            baseGestures[4].SetActive(isGesture);
        }
    }

    /// <summary>
    /// Check if a Point Independent gesture is
    /// being performed by the user.
    /// </summary>
    /// <param name="isGesture">bool - wheter the gesture is being performed.</param>
    public void DisplayPointIndependentGesture(bool isGesture, bool isSqueeze, bool isPointA, GameObject[] baseGestures)
    {
        if (isSqueeze || isPointA)
        {
            baseGestures[1].SetActive(false);
        }
        else
        {
            baseGestures[1].SetActive(isGesture);
        }
    }

    /// <summary>
    /// Check if a pinch trackpad gesture is
    /// being performed by the user.
    /// </summary>
    /// <param name="isGesture">bool - wheter the gesture is being performed.false</param>
    public void DisplayPinchThumbFingerGesture(bool isGesture, bool isSqueeze, bool isPointA, bool isPinchThumbFinger, GameObject[] baseGestures)
    {
        if (isSqueeze || isPointA || isPinchThumbFinger)
        {
            baseGestures[0].SetActive(false);
        }
        else
        {
            baseGestures[0].SetActive(isGesture);
        }
    }

    /// <summary>
    /// Check if a punch thumbfinger gesture is
    /// being performed by the user.
    /// </summary>
    /// <param name="isGesture">bool - wheter the gesture is being performed.false</param>
    public void DisplayPinchTrackpadGesture(bool isGesture, bool isSqueeze, bool isPointA, bool isPinchTrackpad, GameObject[] baseGestures)
    {
        if (isSqueeze || isPointA || isPinchTrackpad)
        {
            baseGestures[3].SetActive(false);
        }
        else
        {
            baseGestures[3].SetActive(isGesture);
        }
    }
}
