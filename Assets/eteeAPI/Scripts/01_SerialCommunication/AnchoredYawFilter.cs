using System;
using UnityEngine;

namespace IMUPipeline
{
    /// <summary>
    /// Anchored Yaw Filter for dual-hand IMU tracking.
    ///
    /// Uses "support-hand dynamic anchoring":
    /// - The more static hand serves as the world-frame anchor.
    /// - Yaw is computed relative to this anchor, eliminating cumulative drift.
    /// - Smooth transitions when the anchor switches hands.
    ///
    /// AXIS CONVENTION:
    /// - Z is UP (NWU convention assumed).
    /// - Yaw is rotation around the Z-axis.
    ///
    /// KEY PROPERTIES:
    /// 1) If Left hand moves and Right is held still, the frame locks to Right.
    ///    Left correctly shows motion; Right stays locked at its last angle.
    /// 2) If BOTH hands move identically (e.g. body rotation), the support frame
    ///    rotates WITH the body. Both output 0 change - global rotation is rejected.
    /// </summary>
    public class AnchoredYawFilter
    {
        // --- TUNING CONSTANTS ---
        private const float GYRO_STATIC_THRESH = 15.0f;          // deg/s: Gyro magnitude under this is "static"
        private const float ACCEL_1G_TOL = 0.05f;                // g: Accel magnitude deviation from 1.0g allowed
        private const float HYSTERESIS_MARGIN = 5.0f;            // Score difference required to switch support hand
        private const int N_CONFIRM_SAMPLES = 20;                // Samples to confirm a switch (~200ms at 97Hz)
        private const int T_HOLD_SAMPLES = 50;                   // Min samples to hold reference before switching again
        private const float LPF_ALPHA = 0.25f;                   // IIR output smoothing (1.0 = raw, lower = smoother)
        private const float DRIFT_RATE_EMA = 0.02f;              // EMA alpha for drift rate smoothing

        // --- Reference tracking ---
        private bool refIsLeft;
        private int samplesSinceSwitch;
        private int candidateSamples;

        // Frame offset C: maintains continuity when switching anchor hand.
        private Quaternion C;
        // The absolute "World Anchor" quaternion. Frozen during movement.
        private Quaternion QAnchor;

        // --- Output state ---
        private bool firstRun;
        private float filteredYawL;
        private float filteredYawR;

        // Static baseline drift handling
        private float biasL;
        private float biasR;

        // Per-hand drift compensation with drift-rate prediction
        private float? frozenYawL;   // null = not frozen (hand is moving)
        private float? frozenYawR;
        private float driftOffsetL;
        private float driftOffsetR;
        private float driftRateL;
        private float driftRateR;
        private float staticTimeL;
        private float staticTimeR;
        private float staticDriftAccumL;
        private float staticDriftAccumR;

        // --- Public output properties (read by each eteeDevice) ---
        public float YawL { get; private set; }
        public float YawR { get; private set; }
        public bool IsLStatic { get; private set; }
        public bool IsRStatic { get; private set; }

        // --- Per-hand data buffers (for asynchronous feeding) ---
        private float[] pendingQuatL;
        private Vector3 pendingGyroL;
        private Vector3 pendingAccelL;
        private float pendingDtL;
        private bool hasNewL;

        private float[] pendingQuatR;
        private Vector3 pendingGyroR;
        private Vector3 pendingAccelR;
        private float pendingDtR;
        private bool hasNewR;
        private bool bothHandsSeen; // True once both hands have fed data at least once

        public AnchoredYawFilter()
        {
            refIsLeft = true;
            samplesSinceSwitch = 0;
            candidateSamples = 0;

            C = Quaternion.identity;
            QAnchor = Quaternion.identity;

            firstRun = true;
            filteredYawL = 0f;
            filteredYawR = 0f;

            biasL = 0f;
            biasR = 0f;

            frozenYawL = null;
            frozenYawR = null;
            driftOffsetL = 0f;
            driftOffsetR = 0f;
            driftRateL = 0f;
            driftRateR = 0f;
            staticTimeL = 0f;
            staticTimeR = 0f;
            staticDriftAccumL = 0f;
            staticDriftAccumR = 0f;

            hasNewL = false;
            hasNewR = false;
            bothHandsSeen = false;
        }

        /// <summary>
        /// Feed left hand AHRS data. Triggers dual-hand processing if right data is also available.
        /// </summary>
        public void FeedLeft(float[] ahrsQuat, Vector3 gyroCorrDps, Vector3 accelG, float dt)
        {
            pendingQuatL = ahrsQuat;
            pendingGyroL = gyroCorrDps;
            pendingAccelL = accelG;
            pendingDtL = dt;
            hasNewL = true;

            if (!bothHandsSeen)
            {
                if (hasNewR) bothHandsSeen = true;
                else return; // Wait for right hand to appear before processing
            }

            TryProcess();
        }

        /// <summary>
        /// Feed right hand AHRS data. Triggers dual-hand processing if left data is also available.
        /// </summary>
        public void FeedRight(float[] ahrsQuat, Vector3 gyroCorrDps, Vector3 accelG, float dt)
        {
            pendingQuatR = ahrsQuat;
            pendingGyroR = gyroCorrDps;
            pendingAccelR = accelG;
            pendingDtR = dt;
            hasNewR = true;

            if (!bothHandsSeen)
            {
                if (hasNewL) bothHandsSeen = true;
                else return; // Wait for left hand to appear before processing
            }

            TryProcess();
        }

        private void TryProcess()
        {
            // Process whenever we have data from both hands (use latest from each)
            if (pendingQuatL == null || pendingQuatR == null) return;

            float dt = Mathf.Max(pendingDtL, pendingDtR);
            Process(pendingQuatL, pendingQuatR,
                    pendingGyroL, pendingAccelL,
                    pendingGyroR, pendingAccelR,
                    dt,
                    out float yL, out float yR,
                    out bool sL, out bool sR);

            YawL = yL;
            YawR = yR;
            IsLStatic = sL;
            IsRStatic = sR;
        }

        /// <summary>
        /// Compute a 'how static is this hand' score. LOWER = more stationary.
        /// </summary>
        private float GetStaticScore(Vector3 gyroDps, Vector3 accelG)
        {
            float gMag = gyroDps.magnitude;
            float aMag = accelG.magnitude;
            float aDev = Mathf.Abs(aMag - 1.0f);
            return gMag + 200.0f * aDev;
        }

        /// <summary>
        /// Extract yaw angle (rotation about Z) from quaternion, in degrees. Range [-180, 180].
        /// Uses the float[] quaternion format [w, x, y, z].
        /// </summary>
        public static float ExtractYaw(float[] q)
        {
            float w = q[0], x = q[1], y = q[2], z = q[3];
            float t0 = 2.0f * (w * z + x * y);
            float t1 = 1.0f - 2.0f * (y * y + z * z);
            return Mathf.Atan2(t0, t1) * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Extract pitch angle (rotation about Y) from quaternion, in degrees. Range [-90, 90].
        /// Uses the float[] quaternion format [w, x, y, z].
        /// </summary>
        public static float ExtractPitch(float[] q)
        {
            float w = q[0], x = q[1], y = q[2], z = q[3];
            float val = 2.0f * (w * y - z * x);
            val = Mathf.Clamp(val, -1.0f, 1.0f);
            return Mathf.Asin(val) * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Extract roll angle (rotation about X) from quaternion, in degrees. Range [-180, 180].
        /// Uses the float[] quaternion format [w, x, y, z].
        /// </summary>
        public static float ExtractRoll(float[] q)
        {
            float w = q[0], x = q[1], y = q[2], z = q[3];
            float t0 = 2.0f * (w * x + y * z);
            float t1 = 1.0f - 2.0f * (x * x + y * y);
            return Mathf.Atan2(t0, t1) * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Normalize angle to [-180, 180).
        /// </summary>
        public static float NormalizeAngle(float angle)
        {
            while (angle <= -180.0f) angle += 360.0f;
            while (angle > 180.0f) angle -= 360.0f;
            return angle;
        }

        /// <summary>
        /// Reconstruct a quaternion from Euler angles (ZYX intrinsic convention).
        /// q = Rz(yaw) * Ry(pitch) * Rx(roll)
        /// Returns float[] [w, x, y, z].
        /// </summary>
        public static float[] EulerToQuaternion(float rollDeg, float pitchDeg, float yawDeg)
        {
            float r = rollDeg * Mathf.Deg2Rad * 0.5f;
            float p = pitchDeg * Mathf.Deg2Rad * 0.5f;
            float y = yawDeg * Mathf.Deg2Rad * 0.5f;

            float cr = Mathf.Cos(r), sr = Mathf.Sin(r);
            float cp = Mathf.Cos(p), sp = Mathf.Sin(p);
            float cy = Mathf.Cos(y), sy = Mathf.Sin(y);

            float w = cr * cp * cy + sr * sp * sy;
            float x = sr * cp * cy - cr * sp * sy;
            float yy = cr * sp * cy + sr * cp * sy;
            float z = cr * cp * sy - sr * sp * cy;

            return new float[] { w, x, yy, z };
        }

        // --- Quaternion helper methods (using float[] [w,x,y,z]) ---

        private static float[] QuatNormalize(float[] q)
        {
            float n = Mathf.Sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
            if (n < 1e-8f)
                return new float[] { 1f, 0f, 0f, 0f };
            return new float[] { q[0] / n, q[1] / n, q[2] / n, q[3] / n };
        }

        private static float[] QuatConjugate(float[] q)
        {
            return new float[] { q[0], -q[1], -q[2], -q[3] };
        }

        private static float[] QuatMultiply(float[] a, float[] b)
        {
            float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
            float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
            float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
            float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
            return new float[] { w, x, y, z };
        }

        // Wrappers using Quaternion (Unity) ← → float[]
        private static float[] ToArray(Quaternion q)
        {
            // Unity Quaternion: (x,y,z,w) ; our convention: [w,x,y,z]
            return new float[] { q.w, q.x, q.y, q.z };
        }

        private static Quaternion FromArray(float[] q)
        {
            return new Quaternion(q[1], q[2], q[3], q[0]); // Unity: x,y,z,w
        }

        /// <summary>
        /// Process one frame of dual-hand data.
        /// </summary>
        /// <param name="qL">Left hand quaternion from AHRS [w,x,y,z]</param>
        /// <param name="qR">Right hand quaternion from AHRS [w,x,y,z]</param>
        /// <param name="gyroLDps">Left gyro [gx,gy,gz] in deg/s (after bias correction)</param>
        /// <param name="accLG">Left accel [ax,ay,az] in g</param>
        /// <param name="gyroRDps">Right gyro [gx,gy,gz] in deg/s (after bias correction)</param>
        /// <param name="accRG">Right accel [ax,ay,az] in g</param>
        /// <param name="dt">Time step in seconds</param>
        /// <param name="yawL">Output: anchored yaw for left hand in degrees</param>
        /// <param name="yawR">Output: anchored yaw for right hand in degrees</param>
        /// <param name="isLStatic">Output: whether left hand is static</param>
        /// <param name="isRStatic">Output: whether right hand is static</param>
        public void Process(float[] qL, float[] qR,
                            Vector3 gyroLDps, Vector3 accLG,
                            Vector3 gyroRDps, Vector3 accRG,
                            float dt,
                            out float yawL, out float yawR,
                            out bool isLStatic, out bool isRStatic)
        {
            // Normalize input quaternions
            qL = QuatNormalize(qL);
            qR = QuatNormalize(qR);

            // Convert internal Quaternion fields to float[] for calculations
            float[] cArr = ToArray(C);
            float[] qAnchorArr = ToArray(QAnchor);

            // ---- 1. Evaluate static conditions ----
            float scoreL = GetStaticScore(gyroLDps, accLG);
            float scoreR = GetStaticScore(gyroRDps, accRG);

            float gLMag = gyroLDps.magnitude;
            float aLDev = Mathf.Abs(accLG.magnitude - 1.0f);
            isLStatic = (gLMag < GYRO_STATIC_THRESH) && (aLDev < ACCEL_1G_TOL);

            float gRMag = gyroRDps.magnitude;
            float aRDev = Mathf.Abs(accRG.magnitude - 1.0f);
            isRStatic = (gRMag < GYRO_STATIC_THRESH) && (aRDev < ACCEL_1G_TOL);

            // ---- 2. Anchor switching state machine ----
            if (refIsLeft)
            {
                if (scoreR < scoreL - HYSTERESIS_MARGIN)
                    candidateSamples++;
                else
                    candidateSamples = 0;

                if (candidateSamples >= N_CONFIRM_SAMPLES && samplesSinceSwitch >= T_HOLD_SAMPLES)
                {
                    // Switch anchor to Right hand
                    refIsLeft = false;
                    samplesSinceSwitch = 0;
                    candidateSamples = 0;
                    // C_new = conj(qR) * Q_anchor
                    cArr = QuatMultiply(QuatConjugate(qR), qAnchorArr);
                    C = FromArray(cArr);
                }
            }
            else
            {
                if (scoreL < scoreR - HYSTERESIS_MARGIN)
                    candidateSamples++;
                else
                    candidateSamples = 0;

                if (candidateSamples >= N_CONFIRM_SAMPLES && samplesSinceSwitch >= T_HOLD_SAMPLES)
                {
                    // Switch anchor to Left hand
                    refIsLeft = true;
                    samplesSinceSwitch = 0;
                    candidateSamples = 0;
                    cArr = QuatMultiply(QuatConjugate(qL), qAnchorArr);
                    C = FromArray(cArr);
                }
            }

            samplesSinceSwitch++;

            // ---- 3. Update anchor (only when reference hand is static) ----
            if (refIsLeft)
            {
                if (isLStatic)
                {
                    if (frozenYawL == null)
                    {
                        qAnchorArr = QuatMultiply(qL, cArr);
                        QAnchor = FromArray(qAnchorArr);
                    }
                }
            }
            else
            {
                if (isRStatic)
                {
                    if (frozenYawR == null)
                    {
                        qAnchorArr = QuatMultiply(qR, cArr);
                        QAnchor = FromArray(qAnchorArr);
                    }
                }
            }

            // ---- 4. Project both hands into the anchored world frame ----
            float[] qAnchorInv = QuatConjugate(ToArray(QAnchor));
            float[] qLOut = QuatMultiply(qAnchorInv, qL);
            float[] qROut = QuatMultiply(qAnchorInv, qR);

            float yawLRaw = ExtractYaw(qLOut);
            float yawRRaw = ExtractYaw(qROut);

            // ---- 4b. Per-hand drift compensation with rate prediction ----

            // -- Left hand --
            if (isLStatic)
            {
                if (frozenYawL == null)
                {
                    // === MOVING → STATIC TRANSITION (snap-correct) ===
                    if (!firstRun)
                        driftOffsetL = NormalizeAngle(yawLRaw - filteredYawL);
                    frozenYawL = yawLRaw;
                    staticTimeL = 0f;
                    staticDriftAccumL = 0f;
                }
                else
                {
                    // Still static: measure actual AHRS drift this frame
                    float frameDrift = NormalizeAngle(yawLRaw - frozenYawL.Value);
                    driftOffsetL += frameDrift;
                    frozenYawL = yawLRaw;

                    staticTimeL += dt;
                    staticDriftAccumL += frameDrift;
                    if (staticTimeL > 0.5f)
                    {
                        float measuredRate = staticDriftAccumL / staticTimeL;
                        driftRateL = DRIFT_RATE_EMA * measuredRate + (1.0f - DRIFT_RATE_EMA) * driftRateL;
                    }
                }
            }
            else
            {
                if (frozenYawL != null)
                    frozenYawL = null; // Transition static→moving: unfreeze
                // Moving: predict drift from estimated rate
                driftOffsetL += driftRateL * dt;
            }

            yawLRaw -= driftOffsetL;

            // -- Right hand --
            if (isRStatic)
            {
                if (frozenYawR == null)
                {
                    if (!firstRun)
                        driftOffsetR = NormalizeAngle(yawRRaw - filteredYawR);
                    frozenYawR = yawRRaw;
                    staticTimeR = 0f;
                    staticDriftAccumR = 0f;
                }
                else
                {
                    float frameDrift = NormalizeAngle(yawRRaw - frozenYawR.Value);
                    driftOffsetR += frameDrift;
                    frozenYawR = yawRRaw;

                    staticTimeR += dt;
                    staticDriftAccumR += frameDrift;
                    if (staticTimeR > 0.5f)
                    {
                        float measuredRate = staticDriftAccumR / staticTimeR;
                        driftRateR = DRIFT_RATE_EMA * measuredRate + (1.0f - DRIFT_RATE_EMA) * driftRateR;
                    }
                }
            }
            else
            {
                if (frozenYawR != null)
                    frozenYawR = null;
                driftOffsetR += driftRateR * dt;
            }

            yawRRaw -= driftOffsetR;

            // ---- 5. Low-pass filter with anti-jump unwrapping ----
            if (firstRun)
            {
                filteredYawL = yawLRaw;
                filteredYawR = yawRRaw;
                firstRun = false;
            }

            // Unwrap: bring raw value close to current filtered value
            yawLRaw = filteredYawL + NormalizeAngle(yawLRaw - filteredYawL);
            yawRRaw = filteredYawR + NormalizeAngle(yawRRaw - filteredYawR);

            // IIR low-pass
            filteredYawL = LPF_ALPHA * yawLRaw + (1.0f - LPF_ALPHA) * filteredYawL;
            filteredYawR = LPF_ALPHA * yawRRaw + (1.0f - LPF_ALPHA) * filteredYawR;

            // ---- 6. Final output (bias applied via ResetYaw) ----
            yawL = filteredYawL - biasL;
            yawR = filteredYawR - biasR;
        }

        /// <summary>
        /// Reset yaw bias to current filtered values (zero the yaw output).
        /// </summary>
        public void ResetBias()
        {
            biasL = filteredYawL;
            biasR = filteredYawR;
        }
    }
}
