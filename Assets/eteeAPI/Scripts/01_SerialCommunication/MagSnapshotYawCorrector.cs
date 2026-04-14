using System;
using System.Collections.Generic;
using UnityEngine;

namespace IMUPipeline
{
    /// <summary>
    /// Snapshot-based magnetic yaw drift corrector.
    /// 
    /// Instead of continuous tracking, this compares discrete "snapshots" of magnetic
    /// heading taken during confirmed static periods before and after movement.
    /// 
    /// State machine:
    ///   IDLE → (static detected) → COLLECTING_BASELINE → (movement detected) → MOVING
    ///   MOVING → (static detected) → COLLECTING_POST → (enough samples) → CORRECTING → IDLE
    /// 
    /// Uses median filtering on collected heading samples for noise robustness.
    /// Only applies correction when the detected drift exceeds a minimum threshold.
    /// 
    /// Duplicate filtering: MLX90393 runs at ~57Hz vs IMU 97Hz, so ~42% of frames
    /// carry stale mag data. Only unique heading changes are collected.
    /// </summary>
    public class MagSnapshotYawCorrector
    {
        public enum State
        {
            WaitingForCalibration,  // MagCalibrator not ready yet
            CollectingBaseline,     // Collecting static heading samples as baseline
            Ready,                  // Baseline established, waiting for movement
            Moving,                 // Movement detected, not collecting
            CollectingPost,         // Movement ended, collecting post-movement heading
            Correcting              // Applying drift correction
        }

        public State CurrentState { get; private set; }

        // --- Configuration ---
        private const int STATIC_CONFIRM_FRAMES = 200;     // ~2 seconds at 97Hz to confirm static
        private const int SAMPLES_NEEDED = 200;             // ~3.5 seconds of UNIQUE mag samples at ~57Hz (was 150)
        private const float MOVEMENT_THRESHOLD = 5f;        // °/s gyro magnitude to detect movement
        private const float ACCEL_MOVEMENT_THRESHOLD = 0.15f; // g deviation from 1g to confirm real movement
        private const int GYRO_SUSTAINED_FRAMES = 30;   // ~300ms of sustained gyro > threshold = real movement
        private const float MIN_DRIFT_TO_CORRECT = 15f;     // Minimum drift in degrees to bother correcting (was 3°)
        private const float MAX_VALID_DRIFT = 45f;           // If drift > this, mag data is suspect → skip (was 90°)
        private const float HEADING_DUP_EPSILON = 0.01f;    // Tolerance for duplicate heading detection

        // --- Sample collection ---
        private List<float> headingSamples;
        private int staticFrameCount;
        private float lastMagHeading;         // Previous mag heading for duplicate detection
        private bool hasLastMagHeading;       // Whether we have a previous heading to compare
        private int skippedDuplicates;        // Count of skipped duplicate frames (for diagnostics)
        private int gyroMovingCount;           // Consecutive frames with gyro > threshold (for sustained movement detection)

        // --- Baseline snapshot ---
        private float baselineHeading;   // Median heading before movement
        private float baselineAhrsYaw;   // AHRS yaw at baseline capture

        // --- Post-movement snapshot ---
        private float postHeading;       // Median heading after movement
        private float postAhrsYaw;       // AHRS yaw after movement

        // --- Correction state ---
        private float remainingCorrection;  // Degrees of correction still to apply
        public float RemainingCorrection => remainingCorrection;
        public float DetectedDrift { get; private set; }

        // --- Tilt compensation inputs (set each frame) ---
        private float currentMagHeading;
        private float currentAhrsYaw;
        private float currentGyroMag;
        private bool currentMagReliable;

        public MagSnapshotYawCorrector()
        {
            headingSamples = new List<float>(SAMPLES_NEEDED + 50);
            CurrentState = State.WaitingForCalibration;
            staticFrameCount = 0;
            remainingCorrection = 0f;
            DetectedDrift = 0f;
            lastMagHeading = 0f;
            hasLastMagHeading = false;
            skippedDuplicates = 0;
            gyroMovingCount = 0;
        }

        /// <summary>
        /// Reset to baseline collection state. Call when orientation is reset
        /// (e.g. "Reset Rotation" button) to avoid stale baseline causing false drift.
        /// </summary>
        public void ResetBaseline()
        {
            CurrentState = State.CollectingBaseline;
            staticFrameCount = 0;
            headingSamples.Clear();
            remainingCorrection = 0f;
            DetectedDrift = 0f;
            gyroMovingCount = 0;
            hasLastMagHeading = false;
        }

        /// <summary>
        /// Call every frame with current sensor data. Returns the yaw correction
        /// (in degrees) to apply THIS frame, or 0 if no correction needed.
        /// </summary>
        public float Update(float magHeading, float ahrsYaw, float gyroMagDps,
                           bool magCalibrated, bool magReliable, bool isSettling,
                           string handLabel, float accelDeviation = 0f)
        {
            currentMagHeading = magHeading;
            currentAhrsYaw = ahrsYaw;
            currentGyroMag = gyroMagDps;
            currentMagReliable = magReliable;

            if (!magCalibrated)
            {
                CurrentState = State.WaitingForCalibration;
                return 0f;
            }

            // Detect duplicate mag readings (~42% of frames at 57Hz mag vs 97Hz IMU)
            bool isUniqueReading = true;
            if (hasLastMagHeading)
            {
                float headingDelta = Mathf.Abs(NormalizeAngle(magHeading - lastMagHeading));
                isUniqueReading = headingDelta > HEADING_DUP_EPSILON;
            }
            lastMagHeading = magHeading;
            hasLastMagHeading = true;

            float correctionThisFrame = 0f;

            switch (CurrentState)
            {
                case State.WaitingForCalibration:
                    // Transition to collecting baseline once calibrated
                    CurrentState = State.CollectingBaseline;
                    staticFrameCount = 0;
                    headingSamples.Clear();
                    Debug.Log($"[MagSnap-{handLabel}] Calibration ready, collecting baseline...");
                    break;

                case State.CollectingBaseline:
                    if (gyroMagDps > MOVEMENT_THRESHOLD)
                    {
                        // Movement during baseline collection → restart
                        staticFrameCount = 0;
                        headingSamples.Clear();
                    }
                    else
                    {
                        staticFrameCount++;
                        if (staticFrameCount > STATIC_CONFIRM_FRAMES && magReliable && isUniqueReading)
                        {
                            headingSamples.Add(magHeading);
                            if (headingSamples.Count >= SAMPLES_NEEDED)
                            {
                                baselineHeading = MedianAngle(headingSamples);
                                baselineAhrsYaw = ahrsYaw;
                                CurrentState = State.Ready;
                                skippedDuplicates = 0;
                                Debug.Log($"[MagSnap-{handLabel}] ★ Baseline captured: " +
                                          $"magH={baselineHeading:F1} ahrsY={baselineAhrsYaw:F1} " +
                                          $"(from {headingSamples.Count} unique samples)");
                                headingSamples.Clear();
                            }
                        }
                        else if (!isUniqueReading)
                        {
                            skippedDuplicates++;
                        }
                    }
                    break;

                case State.Ready:
                    // Movement detection with two paths:
                    //   Fast: gyro + accel both exceed threshold (immediate)
                    //   Slow: gyro sustained for 30 frames without accel (handles pure yaw rotation)
                    if (gyroMagDps > MOVEMENT_THRESHOLD)
                    {
                        gyroMovingCount++;
                        if (accelDeviation > ACCEL_MOVEMENT_THRESHOLD || gyroMovingCount >= GYRO_SUSTAINED_FRAMES)
                        {
                            CurrentState = State.Moving;
                            gyroMovingCount = 0;
                        }
                    }
                    else
                    {
                        gyroMovingCount = 0;  // Reset if gyro drops below threshold
                        if (magReliable && isUniqueReading)
                        {
                            // While idle, slowly update baseline with EMA to handle environmental drift
                            float diff = NormalizeAngle(magHeading - baselineHeading);
                            baselineHeading += 0.002f * diff;
                            baselineAhrsYaw = ahrsYaw;  // Keep baseline ahrsYaw current
                        }
                    }
                    break;

                case State.Moving:
                    if (gyroMagDps <= MOVEMENT_THRESHOLD)
                    {
                        staticFrameCount = 0;
                        headingSamples.Clear();
                        CurrentState = State.CollectingPost;
                    }
                    break;

                case State.CollectingPost:
                    if (gyroMagDps > MOVEMENT_THRESHOLD)
                    {
                        // Movement resumed → go back to moving
                        CurrentState = State.Moving;
                        headingSamples.Clear();
                    }
                    else
                    {
                        staticFrameCount++;
                        if (staticFrameCount > STATIC_CONFIRM_FRAMES && magReliable && isUniqueReading)
                        {
                            headingSamples.Add(magHeading);
                            if (headingSamples.Count >= SAMPLES_NEEDED)
                            {
                                postHeading = MedianAngle(headingSamples);
                                postAhrsYaw = ahrsYaw;

                                // Compute drift
                                float magRotation = NormalizeAngle(postHeading - baselineHeading);
                                float ahrsRotation = NormalizeAngle(postAhrsYaw - baselineAhrsYaw);
                                float drift = NormalizeAngle(magRotation - ahrsRotation);

                                DetectedDrift = drift;
                                headingSamples.Clear();

                                if (Mathf.Abs(drift) > MIN_DRIFT_TO_CORRECT &&
                                    Mathf.Abs(drift) < MAX_VALID_DRIFT)
                                {
                                    remainingCorrection = drift;
                                    CurrentState = State.Correcting;
                                    skippedDuplicates = 0;
                                    Debug.Log($"[MagSnap-{handLabel}] ★ Drift detected: {drift:F1}° " +
                                              $"(magRot={magRotation:F1} ahrsRot={ahrsRotation:F1}) " +
                                              $"Correcting...");
                                }
                                else
                                {
                                    if (Mathf.Abs(drift) >= MAX_VALID_DRIFT)
                                    {
                                        Debug.Log($"[MagSnap-{handLabel}] Drift={drift:F1}° exceeds max " +
                                                  $"({MAX_VALID_DRIFT}°) — mag unreliable, skipping.");
                                    }
                                    else
                                    {
                                        Debug.Log($"[MagSnap-{handLabel}] Drift={drift:F1}° below threshold " +
                                                  $"({MIN_DRIFT_TO_CORRECT}°) — no correction needed.");
                                    }

                                    // Re-establish baseline at current position
                                    baselineHeading = postHeading;
                                    baselineAhrsYaw = postAhrsYaw;
                                    CurrentState = State.Ready;
                                }
                            }
                        }
                    }
                    break;

                case State.Correcting:
                    if (gyroMagDps > MOVEMENT_THRESHOLD)
                    {
                        // Movement during correction → abort, go back to moving
                        Debug.Log($"[MagSnap-{handLabel}] Correction interrupted by movement " +
                                  $"(remaining={remainingCorrection:F1}°)");
                        remainingCorrection = 0f;
                        CurrentState = State.Moving;
                    }
                    else if (Mathf.Abs(remainingCorrection) < 0.1f)
                    {
                        // Correction complete
                        Debug.Log($"[MagSnap-{handLabel}] ✓ Correction complete.");
                        remainingCorrection = 0f;

                        // Re-establish baseline at corrected position
                        baselineHeading = postHeading;
                        baselineAhrsYaw = ahrsYaw;  // Use current (corrected) ahrsYaw
                        CurrentState = State.Ready;
                    }
                    else
                    {
                        // Apply gradual correction: 0.3°/frame ≈ 29°/sec at 97Hz
                        float maxStep = isSettling ? 0.5f : 0.3f;
                        correctionThisFrame = Mathf.Clamp(remainingCorrection, -maxStep, maxStep);
                        remainingCorrection -= correctionThisFrame;
                    }
                    break;
            }

            return correctionThisFrame;
        }

        /// <summary>
        /// Reset the corrector (e.g., when user presses Reset Rotation).
        /// </summary>
        public void Reset()
        {
            CurrentState = State.CollectingBaseline;
            staticFrameCount = 0;
            headingSamples.Clear();
            remainingCorrection = 0f;
            DetectedDrift = 0f;
            lastMagHeading = 0f;
            hasLastMagHeading = false;
            skippedDuplicates = 0;
        }

        // --- Helpers ---

        private static float NormalizeAngle(float angle)
        {
            while (angle > 180f) angle -= 360f;
            while (angle <= -180f) angle += 360f;
            return angle;
        }

        /// <summary>
        /// Compute the median of a list of angles (handles wraparound).
        /// Uses the circular median: finds the angle that minimizes total angular distance.
        /// For simplicity, uses the "unwrap + standard median" approach.
        /// </summary>
        private static float MedianAngle(List<float> angles)
        {
            if (angles.Count == 0) return 0f;
            if (angles.Count == 1) return angles[0];

            // Unwrap angles relative to the first sample
            float reference = angles[0];
            List<float> unwrapped = new List<float>(angles.Count);
            for (int i = 0; i < angles.Count; i++)
            {
                float diff = angles[i] - reference;
                while (diff > 180f) diff -= 360f;
                while (diff <= -180f) diff += 360f;
                unwrapped.Add(reference + diff);
            }

            // Sort and take median
            unwrapped.Sort();
            int mid = unwrapped.Count / 2;
            float median;
            if (unwrapped.Count % 2 == 0)
                median = (unwrapped[mid - 1] + unwrapped[mid]) / 2f;
            else
                median = unwrapped[mid];

            // Re-normalize to [-180, 180]
            return NormalizeAngle(median);
        }
    }
}
