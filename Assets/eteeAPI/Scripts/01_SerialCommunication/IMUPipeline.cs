using System;
using UnityEngine;
using AHRS;

namespace IMUPipeline
{
    /// <summary>
    /// Complete IMU processing pipeline for dual-hand tracking.
    ///
    /// Stage 1: Axis alignment + dynamic gyro bias estimation (EMA + ZUPT)
    /// Stage 2: 6-axis Madgwick AHRS fusion (no magnetometer)
    /// Stage 3: Anchored Yaw Filter for drift-free yaw
    ///
    /// Outputs stable Pitch, Roll, Yaw (degrees) and reconstructed quaternions for both hands.
    ///
    /// Usage:
    ///     pipeline = new IMUPipeline();
    ///     pipeline.UpdateLeft(accelRaw, gyroRaw);
    ///     pipeline.UpdateRight(accelRaw, gyroRaw);
    ///     var (pitch, roll, yaw) = pipeline.GetAngles("left");
    ///     Quaternion q = pipeline.GetQuaternion("left");
    /// </summary>
    public class IMUPipeline
    {
        // Sensor conversion constants (etee hardware: ±4g accel, ±2000dps gyro)
        private const float ACCEL_SCALE = 4.0f / 32768.0f;           // raw int16 → g
        private const float GYRO_SCALE_DPS = 2000.0f / 32768.0f;     // raw int16 → deg/s
        private const float GYRO_DPS_TO_RPS = Mathf.PI / 180.0f;     // deg/s → rad/s

        // Maximum time (seconds) to wait for the other hand before processing solo.
        private const float PAIR_TIMEOUT = 0.050f;

        private bool flipRightXY;

        // ---- Stage 1: Gyro bias estimators ----
        private GyroBiasEstimator biasL;
        private GyroBiasEstimator biasR;

        // ---- Stage 2: AHRS instances (Madgwick 6-axis) ----
        private MadgwickAHRS ahrsL;
        private MadgwickAHRS ahrsR;

        // ---- Stage 3: Anchored Yaw Filter ----
        private AnchoredYawFilter yawFilter;

        // ---- Time tracking ----
        private float? lastTime;
        private float? lastTimeL;
        private float? lastTimeR;
        private float? lastSeenL;
        private float? lastSeenR;

        // ---- Output storage ----
        public float PitchL { get; private set; }
        public float RollL { get; private set; }
        public float YawL { get; private set; }
        public float PitchR { get; private set; }
        public float RollR { get; private set; }
        public float YawR { get; private set; }

        // Display quaternions (Unity Quaternion) with anchored yaw
        public Quaternion QuaternionL { get; private set; }
        public Quaternion QuaternionR { get; private set; }

        // Raw AHRS quaternions for single-hand fallback
        private float[] ahrsQuatL;
        private float[] ahrsQuatR;
        private bool hasDual;

        // Static freeze for pitch/roll
        private float lastPitchL, lastRollL;
        private float lastPitchR, lastRollR;
        private bool firstRunL = true;
        private bool firstRunR = true;

        // ---- Frame pairing buffer ----
        private (Vector3 accel, Vector3 gyro)? pendingLeft;
        private (Vector3 accel, Vector3 gyro)? pendingRight;

        // Debug
        private int debugCount = 0;

        public IMUPipeline(bool flipRightXY = false, float gyroBiasAlpha = 0.00001f, float? madgwickBeta = null)
        {
            this.flipRightXY = flipRightXY;

            // Stage 1
            biasL = new GyroBiasEstimator(gyroBiasAlpha);
            biasR = new GyroBiasEstimator(gyroBiasAlpha);

            // Stage 2
            float samplePeriod = 1.0f / 97.0f;
            float beta = madgwickBeta ?? 0.0315f;
            ahrsL = new MadgwickAHRS(samplePeriod, beta);
            ahrsR = new MadgwickAHRS(samplePeriod, beta);

            // Stage 3
            yawFilter = new AnchoredYawFilter();

            // Output
            QuaternionL = Quaternion.identity;
            QuaternionR = Quaternion.identity;
            ahrsQuatL = new float[] { 1, 0, 0, 0 };
            ahrsQuatR = new float[] { 1, 0, 0, 0 };
        }

        // ================================================================
        // Public API
        // ================================================================

        /// <summary>
        /// Buffer a left hand IMU frame with raw int16 values.
        /// </summary>
        public void UpdateLeft(Vector3 accelRaw, Vector3 gyroRaw)
        {
            pendingLeft = (accelRaw, gyroRaw);
            TryProcess();
        }

        /// <summary>
        /// Buffer a right hand IMU frame with raw int16 values.
        /// </summary>
        public void UpdateRight(Vector3 accelRaw, Vector3 gyroRaw)
        {
            pendingRight = (accelRaw, gyroRaw);
            TryProcess();
        }

        /// <summary>
        /// Get the current stable angles for a hand.
        /// </summary>
        public (float pitch, float roll, float yaw) GetAngles(string hand)
        {
            if (hand == "left")
                return (PitchL, RollL, YawL);
            else if (hand == "right")
                return (PitchR, RollR, YawR);
            return (0f, 0f, 0f);
        }

        /// <summary>
        /// Get the display quaternion for a hand (for 3D visualization).
        /// </summary>
        public Quaternion GetQuaternion(string hand)
        {
            if (hand == "left")
                return QuaternionL;
            else if (hand == "right")
                return QuaternionR;
            return Quaternion.identity;
        }

        /// <summary>
        /// Zero the yaw output for both hands.
        /// </summary>
        public void ResetYaw()
        {
            yawFilter.ResetBias();
        }

        // ================================================================
        // Internal Processing
        // ================================================================

        private void TryProcess()
        {
            float now = Time.realtimeSinceStartup;

            if (pendingLeft != null && pendingRight != null)
            {
                // Both hands ready: run full 3-stage pipeline
                var (accelRawL, gyroRawL) = pendingLeft.Value;
                var (accelRawR, gyroRawR) = pendingRight.Value;
                pendingLeft = null;
                pendingRight = null;

                float dt;
                if (lastTime == null)
                    dt = 1.0f / 97.0f;
                else
                {
                    dt = now - lastTime.Value;
                    dt = Mathf.Clamp(dt, 0.001f, 0.1f);
                }
                lastTime = now;
                lastTimeL = now;
                lastTimeR = now;
                lastSeenL = now;
                lastSeenR = now;
                hasDual = true;

                ProcessFrame(accelRawL, gyroRawL, accelRawR, gyroRawR, dt);
            }
            else if (pendingLeft != null && pendingRight == null)
            {
                // Left arrived, right hasn't.
                bool rightAbsent = (lastSeenR == null) || (now - lastSeenR.Value > 0.2f);
                if (rightAbsent)
                {
                    var (accelRaw, gyroRaw) = pendingLeft.Value;
                    pendingLeft = null;
                    lastSeenL = now;
                    float dt;
                    if (lastTimeL == null)
                        dt = 1.0f / 97.0f;
                    else
                    {
                        dt = now - lastTimeL.Value;
                        dt = Mathf.Clamp(dt, 0.001f, 0.1f);
                    }
                    lastTimeL = now;
                    ProcessSingle(accelRaw, gyroRaw, dt, isRight: false);
                }
            }
            else if (pendingRight != null && pendingLeft == null)
            {
                // Right arrived, left hasn't.
                bool leftAbsent = (lastSeenL == null) || (now - lastSeenL.Value > 0.2f);
                if (leftAbsent)
                {
                    var (accelRaw, gyroRaw) = pendingRight.Value;
                    pendingRight = null;
                    lastSeenR = now;
                    float dt;
                    if (lastTimeR == null)
                        dt = 1.0f / 97.0f;
                    else
                    {
                        dt = now - lastTimeR.Value;
                        dt = Mathf.Clamp(dt, 0.001f, 0.1f);
                    }
                    lastTimeR = now;
                    ProcessSingle(accelRaw, gyroRaw, dt, isRight: true);
                }
            }
        }

        /// <summary>
        /// Stage 1a: Axis alignment for hardware mounting differences.
        /// </summary>
        private void AlignAxes(ref Vector3 accelRaw, ref Vector3 gyroRaw, bool isRight)
        {
            if (isRight && flipRightXY)
            {
                accelRaw.x = -accelRaw.x;
                accelRaw.y = -accelRaw.y;
                gyroRaw.x = -gyroRaw.x;
                gyroRaw.y = -gyroRaw.y;
            }
        }

        /// <summary>
        /// Convert raw int16 sensor values to physical units (g and deg/s).
        /// </summary>
        private void RawToPhysical(Vector3 accelRaw, Vector3 gyroRaw, out Vector3 accelG, out Vector3 gyroDps)
        {
            accelG = accelRaw * ACCEL_SCALE;
            gyroDps = gyroRaw * GYRO_SCALE_DPS;
        }

        /// <summary>
        /// Full pipeline processing for one paired frame.
        /// Stage 1: Pre-processing (axis alignment → unit conversion → gyro bias)
        /// Stage 2: 6-axis Madgwick AHRS (quaternion for pitch/roll)
        /// Stage 3: Anchored Yaw Filter (drift-free yaw)
        /// </summary>
        private void ProcessFrame(Vector3 accelRawL, Vector3 gyroRawL, Vector3 accelRawR, Vector3 gyroRawR, float dt)
        {
            // ========== STAGE 1: PRE-PROCESSING ==========

            // 1a. Axis alignment
            AlignAxes(ref accelRawL, ref gyroRawL, false);
            AlignAxes(ref accelRawR, ref gyroRawR, true);

            // 1b. Convert raw int16 → physical units (g and deg/s)
            RawToPhysical(accelRawL, gyroRawL, out Vector3 accelGL, out Vector3 gyroDpsL);
            RawToPhysical(accelRawR, gyroRawR, out Vector3 accelGR, out Vector3 gyroDpsR);

            // 1c. Dynamic gyro bias correction (in deg/s space)
            Vector3 gyroCorrDpsL = biasL.Update(gyroDpsL, accelGL);
            Vector3 gyroCorrDpsR = biasR.Update(gyroDpsR, accelGR);

            // Convert corrected gyro to rad/s for Madgwick filter
            Vector3 gyroRpsL = gyroCorrDpsL * GYRO_DPS_TO_RPS;
            Vector3 gyroRpsR = gyroCorrDpsR * GYRO_DPS_TO_RPS;

            // ========== STAGE 2: 6-AXIS AHRS FUSION ==========

            ahrsL.SamplePeriod = dt;
            ahrsL.UpdateRelative(gyroRpsL.x, gyroRpsL.y, gyroRpsL.z, accelGL.x, accelGL.y, accelGL.z);
            float[] qL = (float[])ahrsL.Quaternion.Clone();

            ahrsR.SamplePeriod = dt;
            ahrsR.UpdateRelative(gyroRpsR.x, gyroRpsR.y, gyroRpsR.z, accelGR.x, accelGR.y, accelGR.z);
            float[] qR = (float[])ahrsR.Quaternion.Clone();

            ahrsQuatL = qL;
            ahrsQuatR = qR;

            // Extract stable Pitch and Roll directly from AHRS quaternion
            PitchL = AnchoredYawFilter.ExtractPitch(qL);
            RollL = AnchoredYawFilter.ExtractRoll(qL);
            PitchR = AnchoredYawFilter.ExtractPitch(qR);
            RollR = AnchoredYawFilter.ExtractRoll(qR);

            // ========== STAGE 3: ANCHORED YAW FILTER ==========
            yawFilter.Process(qL, qR,
                              gyroCorrDpsL, accelGL,
                              gyroCorrDpsR, accelGR,
                              dt,
                              out float yawLOut, out float yawROut,
                              out bool isLStatic, out bool isRStatic);

            YawL = yawLOut;
            YawR = yawROut;

            // Freeze Pitch and Roll if hands are completely static
            if (isLStatic && !firstRunL)
            {
                PitchL = lastPitchL;
                RollL = lastRollL;
            }
            else
            {
                lastPitchL = PitchL;
                lastRollL = RollL;
                firstRunL = false;
            }

            if (isRStatic && !firstRunR)
            {
                PitchR = lastPitchR;
                RollR = lastRollR;
            }
            else
            {
                lastPitchR = PitchR;
                lastRollR = RollR;
                firstRunR = false;
            }

            // Debug logging
            debugCount++;
            if (debugCount % 100 == 0)
            {
                Debug.Log($"R_Raw: P={PitchR:F1} R={RollR:F1} Y={AnchoredYawFilter.ExtractYaw(qR):F1} | " +
                          $"R_FiltY={YawR:F1} | L_Raw: P={PitchL:F1} R={RollL:F1} Y={AnchoredYawFilter.ExtractYaw(qL):F1}");
            }

            // ========== RECONSTRUCT DISPLAY QUATERNIONS ==========
            // Replace AHRS yaw (which drifts) with anchored yaw (drift-free)
            float[] reconL = AnchoredYawFilter.EulerToQuaternion(RollL, PitchL, YawL);
            float[] reconR = AnchoredYawFilter.EulerToQuaternion(RollR, PitchR, YawR);

            QuaternionL = new Quaternion(reconL[1], reconL[2], reconL[3], reconL[0]);
            QuaternionR = new Quaternion(reconR[1], reconR[2], reconR[3], reconR[0]);
        }

        /// <summary>
        /// Single-hand processing: Stage 1 + Stage 2 only (no anchored yaw).
        /// Used when only one controller is connected.
        /// </summary>
        private void ProcessSingle(Vector3 accelRaw, Vector3 gyroRaw, float dt, bool isRight)
        {
            // Stage 1a: Axis alignment
            AlignAxes(ref accelRaw, ref gyroRaw, isRight);

            // Stage 1b: Convert to physical units
            RawToPhysical(accelRaw, gyroRaw, out Vector3 accelG, out Vector3 gyroDps);

            // Stage 1c: Dynamic gyro bias correction
            GyroBiasEstimator bias = isRight ? biasR : biasL;
            Vector3 gyroCorrDps = bias.Update(gyroDps, accelG);

            // Convert to rad/s
            Vector3 gyroRps = gyroCorrDps * GYRO_DPS_TO_RPS;

            // Stage 2: Madgwick AHRS
            MadgwickAHRS ahrs = isRight ? ahrsR : ahrsL;
            ahrs.SamplePeriod = dt;
            ahrs.UpdateRelative(gyroRps.x, gyroRps.y, gyroRps.z, accelG.x, accelG.y, accelG.z);
            float[] q = (float[])ahrs.Quaternion.Clone();

            float pitch = AnchoredYawFilter.ExtractPitch(q);
            float roll = AnchoredYawFilter.ExtractRoll(q);
            float yaw = AnchoredYawFilter.ExtractYaw(q);

            // Store outputs (no anchored yaw — use raw AHRS yaw)
            if (isRight)
            {
                ahrsQuatR = q;
                PitchR = pitch;
                RollR = roll;
                YawR = yaw;
                QuaternionR = new Quaternion(q[1], q[2], q[3], q[0]);
            }
            else
            {
                ahrsQuatL = q;
                PitchL = pitch;
                RollL = roll;
                YawL = yaw;
                QuaternionL = new Quaternion(q[1], q[2], q[3], q[0]);
            }
        }
    }
}
