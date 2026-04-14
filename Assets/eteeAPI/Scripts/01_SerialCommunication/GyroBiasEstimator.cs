using System;
using System.Collections.Generic;
using UnityEngine;

namespace IMUPipeline
{
    /// <summary>
    /// Dynamic gyroscope bias estimator with ZUPT and movement-aware tracking.
    ///
    /// Three operating modes:
    ///   1. INITIALIZING: Buffer samples while static, compute initial hardware bias.
    ///   2. STATIC (ZUPT): Zero gyro output, refine bias with ultra-slow EMA.
    ///   3. MOVING: Pass through bias-corrected gyro, track bias drift with medium EMA.
    ///
    /// After movement stops, a brief "settling" window applies faster convergence
    /// to quickly correct any residual drift accumulated during movement.
    /// </summary>
    public class GyroBiasEstimator
    {
        private float alphaStatic;       // Ultra-slow EMA for static refinement
        private float alphaMoving;       // Medium EMA for movement tracking
        private float alphaSettling;     // Fast EMA for post-movement settling
        private Vector3 bias;
        private bool initialized;
        private List<Vector3> initBuffer;

        // Settling state: fast correction window after movement stops
        private int settlingCountdown;

        // Static confirmation: require consecutive static frames before ZUPT
        private int staticConfirmCount;

        /// <summary>Whether the estimator is currently in the settling window (just stopped moving).</summary>
        public bool IsSettling { get { return settlingCountdown > 0; } }

        private const int INIT_BUFFER_SIZE = 20;
        private const float INIT_VARIANCE_THRESHOLD = 0.5f;
        private const float STATIC_GYRO_THRESHOLD = 3.0f;
        private const float STATIC_ACCEL_THRESHOLD = 0.08f;
        private const int STATIC_CONFIRM_FRAMES = 10;  // ~100ms at 97Hz

        // Settling window: how many static samples to use fast convergence after stopping
        private const int SETTLING_SAMPLES = 200;     // ~2 seconds at 97Hz — gravity alignment window after movement stops
        public GyroBiasEstimator(float alphaStatic = 0.00001f, float alphaMoving = 0.0005f, float alphaSettling = 0.1f)
        {
            this.alphaStatic = alphaStatic;
            this.alphaMoving = alphaMoving;
            this.alphaSettling = alphaSettling;
            this.bias = Vector3.zero;
            this.initialized = false;
            this.initBuffer = new List<Vector3>();
            this.settlingCountdown = 0;
            this.staticConfirmCount = 0;
        }

        /// <summary>
        /// Update the bias estimator with new sensor data.
        /// Returns the bias-corrected gyro values in deg/s.
        /// When static, returns (0,0,0) to implement ZUPT.
        /// </summary>
        public Vector3 Update(Vector3 gyroDps, Vector3 accelG)
        {
            float aMag = accelG.magnitude;
            float aDev = Mathf.Abs(aMag - 1.0f);
            bool accelIsStatic = (aDev < STATIC_ACCEL_THRESHOLD);

            // ---- INITIALIZATION PHASE ----
            if (!initialized)
            {
                if (accelIsStatic)
                {
                    initBuffer.Add(gyroDps);
                    if (initBuffer.Count >= INIT_BUFFER_SIZE)
                    {
                        Vector3 mean = Vector3.zero;
                        for (int i = 0; i < initBuffer.Count; i++)
                            mean += initBuffer[i];
                        mean /= initBuffer.Count;

                        Vector3 variance = Vector3.zero;
                        for (int i = 0; i < initBuffer.Count; i++)
                        {
                            Vector3 diff = initBuffer[i] - mean;
                            variance += new Vector3(diff.x * diff.x, diff.y * diff.y, diff.z * diff.z);
                        }
                        variance /= initBuffer.Count;

                        float maxVar = Mathf.Max(variance.x, Mathf.Max(variance.y, variance.z));
                        if (maxVar < INIT_VARIANCE_THRESHOLD)
                        {
                            bias = mean;
                            initialized = true;
                            // NOTE: Do NOT trigger settling here. simon_dev does not auto-settle at init.
                            // At init, AHRS quaternion is still (1,0,0,0) — any gravity correction
                            // at this stage risks converging to a flipped state (pitch ≈ 180°).
                            // Settling should only happen after real movement→static transitions.
                            Debug.Log($"GyroBiasEstimator Initialized Bias: ({bias.x:F4}, {bias.y:F4}, {bias.z:F4})");
                        }
                        else
                        {
                            initBuffer.RemoveRange(0, INIT_BUFFER_SIZE / 2);
                        }
                    }
                }
                else
                {
                    initBuffer.Clear();
                }

                // During initialization, pass through raw - zero bias
                return gyroDps - bias;
            }

            // ---- POST-INITIALIZATION: 3 modes ----
            Vector3 corrected = gyroDps - bias;
            float gMagCorrected = corrected.magnitude;
            bool instantStatic = (gMagCorrected < STATIC_GYRO_THRESHOLD) && accelIsStatic;

            if (instantStatic)
                staticConfirmCount++;
            else
                staticConfirmCount = 0;

            // Require N consecutive static frames to confirm truly stopped
            // (prevents direction-change pauses from triggering false ZUPT)
            bool confirmedStatic = staticConfirmCount >= STATIC_CONFIRM_FRAMES;

            if (confirmedStatic)
            {
                if (settlingCountdown > 0)
                {
                    // MODE: SETTLING (just stopped moving)
                    // Use faster EMA to quickly absorb residual drift
                    bias = alphaSettling * gyroDps + (1.0f - alphaSettling) * bias;
                    settlingCountdown--;
                }
                else
                {
                    // MODE: STATIC (fully settled)
                    // Ultra-slow EMA refinement
                    bias = alphaStatic * gyroDps + (1.0f - alphaStatic) * bias;
                }

                // ZUPT: return zero
                return Vector3.zero;
            }
            else
            {
                // MODE: MOVING (or unconfirmed static)
                // Do NOT update bias during movement — gyro signal is dominated by
                // actual rotation, not hardware bias. Updating here would corrupt
                // the bias estimate, causing large drift after fast movements.

                // Reset settling countdown: will activate when movement truly stops
                settlingCountdown = SETTLING_SAMPLES;

                return corrected;
            }
        }
    }
}
