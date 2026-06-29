using System;
using System.Collections.Generic;
using UnityEngine;

namespace IMUPipeline
{
    /// <summary>
    /// Magnetometer calibrator with hard-iron correction, reliability detection,
    /// and built-in diagnostics for validating sensor data quality.
    ///
    /// Hard-Iron Calibration:
    ///   Collects min/max on each axis → offset = (min+max)/2
    ///   Normalized magnitude = |mag - offset| should be roughly constant
    ///
    /// Reliability Detection:
    ///   If calibrated magnitude deviates too far from the running average,
    ///   the reading is flagged as unreliable (soft-iron distortion, nearby metal, etc.)
    ///
    /// Diagnostics:
    ///   Logs raw/calibrated mag data periodically so the user can verify
    ///   whether the magnetometer hardware is actually producing valid data.
    /// </summary>
    public class MagCalibrator
    {
        // --- Calibration state ---
        private Vector3 minValues;
        private Vector3 maxValues;
        private Vector3 hardIronOffset;
        private float expectedMagnitude;  // Running average of |calibrated mag|

        private int sampleCount;
        private bool calibrated;

        // --- Reliability detection ---
        private float magnitudeEMA;       // Exponential moving average of |calibrated|
        private const float MAG_EMA_ALPHA = 0.01f;
        private const float RELIABILITY_TOLERANCE = 0.35f;  // 35% deviation from expected = unreliable

        // --- Configuration ---
        private const int MIN_CALIBRATION_SAMPLES = 200;       // ~2 seconds at 97Hz
        private const float MIN_AXIS_SPREAD = 10.0f;           // Minimum spread on each axis to consider calibration valid
        private const int RECALIBRATION_INTERVAL = 9700;       // ~100 seconds: periodic re-check

        // --- Diagnostics ---
        private int diagnosticCounter;
        private const int DIAGNOSTIC_LOG_INTERVAL = 5000;       // Log every ~50 seconds at 97Hz (was 500)
        private int zeroDataCount;                              // Count how many consecutive zero-readings
        private const int ZERO_DATA_THRESHOLD = 50;             // If 50+ consecutive zeros, magnetometer is likely absent
        private bool magnetometerPresent = true;                // False if we detect no magnetometer hardware

        // --- Sampling rate diagnostics ---
        private Vector3 prevRawMag;                             // Previous raw mag reading for duplicate detection
        private bool hasPrevRawMag;                             // Whether we have a previous reading to compare
        private int totalFrames;                                // Total frames since last rate report
        private int duplicateFrames;                            // Frames where raw mag == previous raw mag
        private int maxConsecutiveDuplicates;                   // Longest streak of duplicates in this window
        private int currentConsecutiveDuplicates;               // Current streak of duplicates
        private const int RATE_REPORT_INTERVAL = 9700;          // Report every ~100 seconds at 97Hz (was 970)
        private const float DUPLICATE_EPSILON = 0.01f;          // Tolerance for "same reading" comparison

        // --- Lock state (prevents offset updates during movement) ---
        private bool isLocked = false;

        // --- Public status ---
        /// <summary>Whether the calibrator has enough data for valid offsets.</summary>
        public bool IsCalibrated => calibrated;

        /// <summary>Whether the magnetometer hardware appears to be present and producing data.</summary>
        public bool IsMagnetometerPresent => magnetometerPresent;

        /// <summary>The number of samples collected so far.</summary>
        public int SampleCount => sampleCount;

        /// <summary>Current hard-iron offset (valid only when IsCalibrated is true).</summary>
        public Vector3 HardIronOffset => hardIronOffset;

        /// <summary>Whether min/max updates are locked (offset frozen).</summary>
        public bool IsLocked => isLocked;

        /// <summary>Lock offset: prevents min/max updates during movement.</summary>
        public void Lock()
        {
            if (!isLocked)
            {
                isLocked = true;
                // Debug.Log("[MagCalibrator] Offset LOCKED (movement detected)");
            }
        }

        /// <summary>Unlock offset: resumes normal min/max tracking.</summary>
        public void Unlock()
        {
            if (isLocked)
            {
                isLocked = false;
                // Debug.Log("[MagCalibrator] Offset UNLOCKED (static resumed)");
            }
        }

        public MagCalibrator()
        {
            minValues = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            maxValues = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            hardIronOffset = Vector3.zero;
            expectedMagnitude = 0f;
            magnitudeEMA = 0f;
            sampleCount = 0;
            calibrated = false;
            diagnosticCounter = 0;
            zeroDataCount = 0;
            prevRawMag = Vector3.zero;
            hasPrevRawMag = false;
            totalFrames = 0;
            duplicateFrames = 0;
            maxConsecutiveDuplicates = 0;
            currentConsecutiveDuplicates = 0;
        }

        /// <summary>
        /// Feed a raw magnetometer reading. Returns the hard-iron-corrected vector.
        /// Also updates calibration and reliability state.
        /// </summary>
        /// <param name="rawMag">Raw magnetometer reading from sensor</param>
        /// <param name="isReliable">Output: whether this reading is considered reliable</param>
        /// <returns>Hard-iron-corrected magnetometer vector</returns>
        public Vector3 Update(Vector3 rawMag, out bool isReliable)
        {
            diagnosticCounter++;

            // ---- Magnetometer presence detection ----
            if (rawMag.sqrMagnitude < 0.001f)
            {
                zeroDataCount++;
                if (zeroDataCount >= ZERO_DATA_THRESHOLD && magnetometerPresent)
                {
                    magnetometerPresent = false;
                    Debug.LogWarning("[MagCalibrator] Magnetometer appears ABSENT — " +
                                    $"{zeroDataCount} consecutive zero readings. " +
                                    "Falling back to 6-axis mode.");
                }
                isReliable = false;
                return Vector3.zero;
            }
            else
            {
                if (zeroDataCount > 0)
                    zeroDataCount = 0;
                if (!magnetometerPresent)
                {
                    magnetometerPresent = true;
                    Debug.Log("[MagCalibrator] Magnetometer data detected — resuming calibration.");
                }
            }

            // ---- Sampling rate detection (duplicate frame tracking) ----
            totalFrames++;
            if (hasPrevRawMag)
            {
                float delta = (rawMag - prevRawMag).sqrMagnitude;
                if (delta < DUPLICATE_EPSILON * DUPLICATE_EPSILON)
                {
                    duplicateFrames++;
                    currentConsecutiveDuplicates++;
                    if (currentConsecutiveDuplicates > maxConsecutiveDuplicates)
                        maxConsecutiveDuplicates = currentConsecutiveDuplicates;
                }
                else
                {
                    currentConsecutiveDuplicates = 0;
                }
            }
            prevRawMag = rawMag;
            hasPrevRawMag = true;

            // Periodic sampling rate report
            if (totalFrames % RATE_REPORT_INTERVAL == 0 && totalFrames > 0)
            {
                float dupPercent = 100f * duplicateFrames / totalFrames;
                float uniqueFrames = totalFrames - duplicateFrames;
                float estimatedMagHz = uniqueFrames / (totalFrames / 97f);  // Assuming IMU runs at 97Hz
                Debug.Log($"[MagCalibrator] RATE REPORT: " +
                          $"{totalFrames} frames, {duplicateFrames} duplicates ({dupPercent:F1}%), " +
                          $"max consecutive dup={maxConsecutiveDuplicates}, " +
                          $"estimated mag rate={estimatedMagHz:F1}Hz (IMU=97Hz)");
                // Reset counters for next window
                totalFrames = 0;
                duplicateFrames = 0;
                maxConsecutiveDuplicates = 0;
            }

            // ---- Update min/max for hard-iron calibration (skip when locked) ----
            if (!isLocked)
            {
                minValues = Vector3.Min(minValues, rawMag);
                maxValues = Vector3.Max(maxValues, rawMag);
            }
            sampleCount++;

            // ---- Check if we have enough spread for valid calibration ----
            Vector3 spread = maxValues - minValues;
            bool hasSpread = spread.x > MIN_AXIS_SPREAD &&
                             spread.y > MIN_AXIS_SPREAD &&
                             spread.z > MIN_AXIS_SPREAD;

            if (!calibrated && sampleCount >= MIN_CALIBRATION_SAMPLES && hasSpread)
            {
                hardIronOffset = (minValues + maxValues) * 0.5f;
                calibrated = true;
                Vector3 sample = rawMag - hardIronOffset;
                magnitudeEMA = sample.magnitude;
                expectedMagnitude = magnitudeEMA;
                Debug.Log($"[MagCalibrator] Calibration complete after {sampleCount} samples. " +
                          $"Offset=({hardIronOffset.x:F1}, {hardIronOffset.y:F1}, {hardIronOffset.z:F1}), " +
                          $"Expected |mag|={expectedMagnitude:F1}");
            }
            else if (calibrated && !isLocked && sampleCount % RECALIBRATION_INTERVAL == 0)
            {
                // Periodic re-calibration of offset
                Vector3 newOffset = (minValues + maxValues) * 0.5f;
                float drift = (newOffset - hardIronOffset).magnitude;
                if (drift > 2.0f)
                {
                    hardIronOffset = newOffset;
                    Debug.Log($"[MagCalibrator] Offset re-calibrated. " +
                              $"New offset=({hardIronOffset.x:F1}, {hardIronOffset.y:F1}, {hardIronOffset.z:F1}), " +
                              $"drift={drift:F1}");
                }
            }

            // ---- Apply hard-iron correction ----
            Vector3 corrected;
            if (calibrated)
            {
                corrected = rawMag - hardIronOffset;
            }
            else
            {
                // Before calibration, use raw data (no offset correction)
                corrected = rawMag;
            }

            // ---- Reliability check ----
            float mag = corrected.magnitude;

            if (calibrated)
            {
                // Update magnitude EMA
                magnitudeEMA = MAG_EMA_ALPHA * mag + (1f - MAG_EMA_ALPHA) * magnitudeEMA;

                // Check if current magnitude is within tolerance of expected
                float deviation = Mathf.Abs(mag - magnitudeEMA) / magnitudeEMA;
                isReliable = deviation < RELIABILITY_TOLERANCE;
            }
            else
            {
                // Not calibrated yet — unreliable by definition
                isReliable = false;
            }

            // ---- Diagnostic logging ----
            if (diagnosticCounter % DIAGNOSTIC_LOG_INTERVAL == 0)
            {
                if (!calibrated)
                {
                    Debug.Log($"[MagCalibrator] DIAG #{diagnosticCounter}: " +
                              $"raw=({rawMag.x:F1},{rawMag.y:F1},{rawMag.z:F1}) " +
                              $"|raw|={rawMag.magnitude:F1} " +
                              $"spread=({spread.x:F1},{spread.y:F1},{spread.z:F1}) " +
                              $"samples={sampleCount}/{MIN_CALIBRATION_SAMPLES} " +
                              $"status=CALIBRATING");
                }
                else
                {
                    Debug.Log($"[MagCalibrator] DIAG #{diagnosticCounter}: " +
                              $"raw=({rawMag.x:F1},{rawMag.y:F1},{rawMag.z:F1}) " +
                              $"cal=({corrected.x:F1},{corrected.y:F1},{corrected.z:F1}) " +
                              $"|cal|={mag:F1} ema={magnitudeEMA:F1} " +
                              $"reliable={isReliable}");
                }
            }

            return corrected;
        }

        /// <summary>
        /// Reset calibration (e.g. if environment changed significantly).
        /// </summary>
        public void Reset()
        {
            minValues = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            maxValues = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            hardIronOffset = Vector3.zero;
            calibrated = false;
            sampleCount = 0;
            magnitudeEMA = 0f;
            expectedMagnitude = 0f;
            Debug.Log("[MagCalibrator] Reset — recalibration needed.");
        }
    }
}
