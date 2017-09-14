
using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    public class TalonSrx : IGadgeteerUartClient
    {
        public enum Faults
        {
            kCurrentFault = 1,
            kTemperatureFault = 2,
            kBusVoltageFault = 4,
            kGateDriverFault = 8,
            /* SRX extensions */
            kFwdLimitSwitch = 0x10,
            kRevLimitSwitch = 0x20,
            kFwdSoftLimit = 0x40,
            kRevSoftLimit = 0x80,
        };

        const int kBrakeOverride_UseDefaultsFromFlash = 0;
        const int kBrakeOverride_OverrideCoast = 1;
        const int kBrakeOverride_OverrideBrake = 2;

        const int kLimitSwitchOverride_UseDefaultsFromFlash = 1;
        const int kLimitSwitchOverride_DisableFwd_DisableRev = 4;
        const int kLimitSwitchOverride_DisableFwd_EnableRev = 5;
        const int kLimitSwitchOverride_EnableFwd_DisableRev = 6;
        const int kLimitSwitchOverride_EnableFwd_EnableRev = 7;

        /* Motion Profile status bits */
        const int kMotionProfileFlag_ActTraj_IsValid = 0x1;
        const int kMotionProfileFlag_HasUnderrun = 0x2;
        const int kMotionProfileFlag_IsUnderrun = 0x4;
        const int kMotionProfileFlag_ActTraj_IsLast = 0x8;
        const int kMotionProfileFlag_ActTraj_VelOnly = 0x10;

        public enum ControlMode
        {
            kPercentVbus = 0,
            kCurrent = 1,
            kSpeed = 2,
            kPosition = 3,
            kVoltage = 4,
            kFollower = 5,
            kMotionProfile = 6,
            kMotionMagic = 7,
        };

        public enum LimitMode
        {
            /** Only use switches for limits */
            kLimitMode_SwitchInputsOnly = 0,
            /** Use both switches and soft limits */
            kLimitMode_SoftPositionLimits = 1,
            /* SRX extensions */
            /** Disable switches and disable soft limits */
            kLimitMode_SrxDisableSwitchInputs = 2,
        };

        public enum NeutralMode
        {
            /** Use the NeutralMode that is set by the jumper wire on the CAN device */
            Jumper = 0,
            /** Stop the motor's rotation by applying a force. */
            Brake = 1,
            /** Do not attempt to stop the motor. Instead allow it to coast to a stop
               without applying resistance. */
            Coast = 2
        };
        public enum FeedbackDevice
        {
            QuadEncoder = 0,
            AnalogPot = 2,
            AnalogEncoder = 3,
            EncRising = 4,
            EncFalling = 5,
            CtreMagEncoder_Relative = 6, //!< Cross The Road Electronics Magnetic Encoder in Relative/Quadrature Mode
            CtreMagEncoder_Absolute = 7, //!< Cross The Road Electronics Magnetic Encoder in Absolute/PulseWidth Mode
            PulseWidth = 8,
        };
        /**
         * Depending on the sensor type, Talon can determine if sensor is plugged in ot not.
         */
        public enum FeedbackDeviceStatus
        {
            FeedbackStatusUnknown = 0,      //!< Sensor status could not be determined.  Not all sensors can do this.
            FeedbackStatusPresent = 1,      //!< Sensor is present and working okay.
            FeedbackStatusNotPresent = 2,   //!< Sensor is not present, not plugged in, not powered, etc...
        };
        public enum StatusFrameRate
        {
            StatusFrameRateGeneral = 0,
            StatusFrameRateFeedback = 1,
            StatusFrameRateQuadEncoder = 2,
            StatusFrameRateAnalogTempVbat = 3,
            StatusFrameRatePulseWidthMeas = 4,
            StatusFrameMotionProfile = 5,
            StatusFrameMotionMagic = 6,
        };
        /**
         * Enumerated types for Motion Control Set Values.
         * When in Motion Profile control mode, these constants are paseed
         * into set() to manipulate the motion profile executer.
         * When changing modes, be sure to read the value back using getMotionProfileStatus()
         * to ensure changes in output take effect before performing buffering actions.
         * Disable will signal Talon to put motor output into neutral drive.
         *   Talon will stop processing motion profile points.  This means the buffer is
         *   effectively disconnected from the executer, allowing the robot to gracefully
         *   clear and push new traj points.  isUnderrun will get cleared.
         *   The active trajectory is also cleared.
         * Enable will signal Talon to pop a trajectory point from it's buffer and process it.
         *   If the active trajectory is empty, Talon will shift in the next point.
         *   If the active traj is empty, and so is the buffer, the motor drive is neutral and
         *   isUnderrun is set.  When active traj times out, and buffer has at least one point,
         *   Talon shifts in next one, and isUnderrun is cleared.  When active traj times out,
         *   and buffer is empty, Talon keeps processing active traj and sets IsUnderrun.
         * Hold will signal Talon keep processing the active trajectory indefinitely.
         *   If the active traj is cleared, Talon will neutral motor drive.  Otherwise
         *    Talon will keep processing the active traj but it will not shift in
         *    points from the buffer.  This means the buffer is  effectively disconnected
         *    from the executer, allowing the robot to gracefully clear and push
         *    new traj points.
         *    isUnderrun is set if active traj is empty, otherwise it is cleared.
         *    isLast signal is also cleared.
         *
         * Typical workflow:
         *   set(Disable),
         *   Confirm Disable takes effect,
         *   clear buffer and push buffer points,
         *   set(Enable) when enough points have been pushed to ensure no underruns,
         *   wait for MP to finish or decide abort,
         *   If MP finished gracefully set(Hold) to hold position servo and disconnect buffer,
         *   If MP is being aborted set(Disable) to neutral the motor and disconnect buffer,
         *   Confirm mode takes effect,
         *   clear buffer and push buffer points, and rinse-repeat.
         */
        public enum SetValueMotionProfile
        {
            Disable = 0,
            Enable = 1,
            Hold = 2,
        };
        /**
         * Motion Profile Trajectory Point
         * This is simply a data transer object.
         */
        public struct TrajectoryPoint
        {
            public float position; //!< The position to servo to.
            public float velocity; //!< The velocity to feed-forward.
                                   /**
                                    * Time in milliseconds to process this point.
                                    * Value should be between 1ms and 255ms.  If value is zero
                                    * then Talon will default to 1ms.  If value exceeds 255ms API will cap it.
                                    */
            public UInt32 timeDurMs;
            /**
             * Which slot to get PIDF gains.
             * PID is used for position servo.
             * F is used as the Kv constant for velocity feed-forward.
             * Typically this is hardcoded to the a particular slot, but you are free
             * gain schedule if need be.
             */
            public UInt32 profileSlotSelect;
            /**
             * Set to true to only perform the velocity feed-forward and not perform
             * position servo.  This is useful when learning how the position servo
             * changes the motor response.  The same could be accomplish by clearing the
             * PID gains, however this is synchronous the streaming, and doesn't require restoing
             * gains when finished.
             *
             * Additionaly setting this basically gives you direct control of the motor output
             * since motor output = targetVelocity X Kv, where Kv is our Fgain.
             * This means you can also scheduling straight-throttle curves without relying on
             * a sensor.
             */
            public bool velocityOnly;
            /**
             * Set to true to signal Talon that this is the final point, so do not
             * attempt to pop another trajectory point from out of the Talon buffer.
             * Instead continue processing this way point.  Typically the velocity
             * member variable should be zero so that the motor doesn't spin indefinitely.
             */
            public bool isLastPoint;
            /**
              * Set to true to signal Talon to zero the selected sensor.
              * When generating MPs, one simple method is to make the first target position zero,
              * and the final target position the target distance from the current position.
              * Then when you fire the MP, the current position gets set to zero.
              * If this is the intent, you can set zeroPos on the first trajectory point.
              *
              * Otherwise you can leave this false for all points, and offset the positions
              * of all trajectory points so they are correct.
              */
            public bool zeroPos;
        };
        /**
         * Motion Profile Status
         * This is simply a data transer object.
         */
        public struct MotionProfileStatus
        {
            /**
             * The available empty slots in the trajectory buffer.
             *
             * The robot API holds a "top buffer" of trajectory points, so your applicaion
             * can dump several points at once.  The API will then stream them into the Talon's
             * low-level buffer, allowing the Talon to act on them.
             */
            public UInt32 topBufferRem;
            /**
             * The number of points in the top trajectory buffer.
             */
            public UInt32 topBufferCnt;
            /**
             * The number of points in the low level Talon buffer.
             */
            public UInt32 btmBufferCnt;
            /**
             * Set if isUnderrun ever gets set.
             * Only is cleared by clearMotionProfileHasUnderrun() to ensure
             * robot logic can react or instrument it.
             * @see clearMotionProfileHasUnderrun()
             */
            public bool hasUnderrun;
            /**
             * This is set if Talon needs to shift a point from its buffer into
             * the active trajectory point however the buffer is empty. This gets cleared
             * automatically when is resolved.
             */
            public bool isUnderrun;
            /**
             * True if the active trajectory point has not empty, false otherwise.
             * The members in activePoint are only valid if this signal is set.
             */
            public bool activePointValid;
            /**
             * The number of points in the low level Talon buffer.
             */
            public TrajectoryPoint activePoint;
            /**
             * The current output mode of the motion profile executer (disabled, enabled, or hold).
             * When changing the set() value in MP mode, it's important to check this signal to
             * confirm the change takes effect before interacting with the top buffer.
             */
            public SetValueMotionProfile outputEnable;
        };


        // Values for various modes as is sent in the CAN packets for the Talon.
        private enum TalonControlMode
        {
            kThrottle = 0,
            kFollowerMode = 5,
            kVoltageMode = 4,
            kPositionMode = 1,
            kSpeedMode = 2,
            kCurrentMode = 3,
            kMotionProfileMode = 6,
            kMotionMagic = 7,
            kDisabled = 15
        };

        private CTRE.LowLevel_TalonSrx m_impl;
        private uint m_profile = 0;  // Profile from CANTalon to use. Set to zero until we can
                                     // actually test this.

        private bool m_controlEnabled = true;
        private ControlMode m_controlMode = ControlMode.kPercentVbus;
        private TalonControlMode m_sendMode;

        private float m_setPoint = 0;
        /**
         * Encoder CPR, counts per rotations, also called codes per revoluion.
         * Default value of zero means the API behaves as it did during the 2015 season, each position
         * unit is a single pulse and there are four pulses per count (4X).
         * Caller can use ConfigEncoderCodesPerRev to set the quadrature encoder CPR.
         */
        private UInt32 m_codesPerRev = 0;
        /**
         * Number of turns per rotation.  For example, a 10-turn pot spins ten full rotations from
         * a wiper voltage of zero to 3.3 volts.  Therefore knowing the
         * number of turns a full voltage sweep represents is necessary for calculating rotations
         * and velocity.
         * A default value of zero means the API behaves as it did during the 2015 season, there are 1024
         * position units from zero to 3.3V.
         */
        private UInt32 m_numPotTurns = 0;
        /**
         * Although the Talon handles feedback selection, caching the feedback selection is helpful at the API level
         * for scaling into rotations and RPM.
         */
        private FeedbackDevice m_feedbackDevice = FeedbackDevice.QuadEncoder;

        private const int kDelayForSolicitedSignalsMs = 4;

        bool m_isInverted;

        UInt32 _deviceNumber;
        /**
		 * Number of adc engineering units per 0 to 3.3V sweep.
		 * This is necessary for scaling Analog Position in rotations/RPM.
		 */
        const float kNativeAdcUnitsPerRotation = 1024.0f;
        /**
		 * Number of pulse width engineering units per full rotation.
		 * This is necessary for scaling Pulse Width Decoded Position in rotations/RPM.
		 */
        const float kNativePwdUnitsPerRotation = 4096.0f;
        /**
		 * Number of minutes per 100ms unit.  Useful for scaling velocities
		 * measured by Talon's 100ms timebase to rotations per minute.
		 */
        const float kMinutesPer100msUnit = 1.0f / 600.0f;

        const int CAN_OK = 0;

        private int _lastStatus = 0;
        /**
         * Constructor for the CANTalon device.
         * @param deviceNumber The CAN ID of the Talon SRX
         * @param externalEnable pass true to prevent sending enable frames.
 		 *  	This can be useful when having one device enable the Talon, and
		 * 		another to control it.
         */
        public TalonSrx(int deviceNumber, bool externalEnable = false)
        {
            _deviceNumber = (UInt32)deviceNumber;
            m_impl = new CTRE.LowLevel_TalonSrx((ushort)deviceNumber, externalEnable);
            ApplyControlMode(m_controlMode);
            m_impl.SetProfileSlotSelect(m_profile);
        }
        /// <summary>
        /// Get the device number this Talon was constructed with.
        /// </summary>
        /// <returns>Device number</returns>
        public UInt32 GetDeviceNumber()
        {
            return _deviceNumber;
        }
        /**
         * Gets the current status of the Talon (usually a sensor value).
         *
         * In Current mode: returns output current.
         * In Speed mode: returns current speed.
         * In Position mode: returns current sensor position.
         * In PercentVbus and Follower modes: returns current applied throttle.
         *
         * @return The current sensor value of the Talon.
         */
        public float Get()
        {
            int value;
            switch (m_controlMode)
            {
                case ControlMode.kVoltage:
                    return GetOutputVoltage();
                case ControlMode.kCurrent:
                    return GetOutputCurrent();
                case ControlMode.kSpeed:
                    m_impl.GetSensorVelocity(out value);
                    return ScaleNativeUnitsToRpm(m_feedbackDevice, value);
                case ControlMode.kPosition:
                    m_impl.GetSensorPosition(out value);
                    return ScaleNativeUnitsToRotations(m_feedbackDevice, value);
                case ControlMode.kPercentVbus:
                case ControlMode.kFollower:
                default:
                    m_impl.GetAppliedThrottle(out value);
                    return (float)value / 1023.0f;
                case ControlMode.kMotionMagic:
                    m_impl.GetSensorPosition(out value);
                    return ScaleNativeUnitsToRotations(m_feedbackDevice, value);
            }
        }
        public int GetLastStatus()
        {
            return _lastStatus;
        }
		private int HandleStatus(int status)
		{
			/* error handler */
			if (status != 0)
			{
				Reporting.SetError(status, Reporting.getHALErrorMessage(status));
			}
			/* mirror last status */
			_lastStatus = status;
            return _lastStatus;
        }
        /**
         * Sets the appropriate output on the talon, depending on the mode.
         *
         * In PercentVbus, the output is between -1.0 and 1.0, with 0.0 as stopped.
         * In Voltage mode, output value is in volts.
         * In Current mode, output value is in amperes.
         * In Speed mode, output value is in position change / 100ms.
         * In Position mode, output value is in encoder ticks or an analog value,
         *   depending on the sensor.
         * In Follower mode, the output value is the integer device ID of the talon to
         * duplicate.
         *
         * @param outputValue The setpoint value, as described above.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public void Set(float value)
        {
            /* feed safety helper since caller just updated our output */
            if (m_controlEnabled)
            {
                m_setPoint = value;  /* cache set point for GetSetpoint() */
                int status = 0;
                switch (m_controlMode)
                {
                    case ControlMode.kPercentVbus:
                        {
                            m_impl.Set(m_isInverted ? -value : value, (int)m_sendMode);
                            status = 0;
                        }
                        break;
                    case ControlMode.kFollower:
                        {
                            status = m_impl.SetDemand((int)value, (int)m_sendMode);
                        }
                        break;
                    case ControlMode.kVoltage:
                        {
                            // Voltage is an 8.8 fixed point number.
                            int volts = (int)((m_isInverted ? -value : value) * 256);
                            status = m_impl.SetDemand(volts, (int)m_sendMode);
                        }
                        break;
                    case ControlMode.kSpeed:
                        /* if the caller has provided scaling info, apply it */
                        status = m_impl.SetDemand24(ScaleVelocityToNativeUnits(m_feedbackDevice, m_isInverted ? -value : value), (int)m_sendMode);
                        break;
                    case ControlMode.kPosition:
                        status = m_impl.SetDemand(ScaleRotationsToNativeUnits(m_feedbackDevice, value), (int)m_sendMode);
                        break;
                    case ControlMode.kCurrent:
                        {
                            float milliamperes = (m_isInverted ? -value : value) * 1000.0f; /* mA*/
                            status = m_impl.SetDemand((int)milliamperes, (int)m_sendMode);
                        }
                        break;
                    case ControlMode.kMotionProfile:
                        {
                            status = m_impl.SetDemand((int)value, (int)m_sendMode);
                        }
                        break;
                    case ControlMode.kMotionMagic:
                        status = m_impl.SetDemand(ScaleRotationsToNativeUnits(m_feedbackDevice, value), (int)m_sendMode);
                        break;
                    default:
                        Debug.Print("The CAN Talon does not support this control mode.");
                        break;
                }
				HandleStatus(status);
            }
        }

        /**
         * Sets the setpoint to value. Equivalent to Set().
         */
        public void SetSetpoint(float value) { Set(value); }

        /**
         * Resets the integral term and disables the controller.
         */
        public void Reset()
        {
            ClearIaccum();
            Disable();
        }

        /**
         * Disables control of the talon, causing the motor to brake or coast
         * depending on its mode (see the Talon SRX Software Reference manual
         * for more information).
         */
        public void Disable()
        {
            m_impl.SetModeSelect((int)TalonControlMode.kDisabled);
            m_controlEnabled = false;
        }

        /**
         * Enables control of the Talon, allowing the motor to move.
         */
        public void EnableControl()
        {
            SetControlMode(m_controlMode);
            m_controlEnabled = true;
        }

        /**
         * Enables control of the Talon, allowing the motor to move.
         */
        public void Enable() { EnableControl(); }

        /**
         * @return Whether the Talon is currently enabled.
         */
        public bool IsControlEnabled() { return m_controlEnabled; }

        /**
         * @return Whether the Talon is currently enabled.
         */
        public bool IsEnabled() { return IsControlEnabled(); }

        /**
         * @param p Proportional constant to use in PID loop.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int SetP(uint slotIdx, float p, uint timeoutMs = 0)
        {
            int status = m_impl.SetPgain(slotIdx, p, timeoutMs);
            return HandleStatus(status);
        }


        /**
         * Set the integration constant of the currently selected profile.
         *
         * @param i Integration constant for the currently selected PID profile.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int SetI(uint slotIdx, float i, uint timeoutMs = 0)
        {
            int status = m_impl.SetIgain(slotIdx, i, timeoutMs);
            return HandleStatus(status);
        }

        /**
         * Set the derivative constant of the currently selected profile.
         *
         * @param d Derivative constant for the currently selected PID profile.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int SetD(uint slotIdx, float d, uint timeoutMs = 0)
        {
            int status = m_impl.SetDgain(slotIdx, d, timeoutMs);
            return HandleStatus(status);
        }
        /**
         * Set the feedforward value of the currently selected profile.
         *
         * @param f Feedforward constant for the currently selected PID profile.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int SetF(uint slotIdx, float f, uint timeoutMs = 0)
        {
            int status = m_impl.SetFgain(slotIdx, f, timeoutMs);
            return HandleStatus(status);
        }
        /**
         * Set the Izone to a nonzero value to auto clear the integral accumulator
         *     when the absolute value of CloseLoopError exceeds Izone.
         *
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int SetIzone(uint slotIdx, UInt32 iz, uint timeoutMs = 0)
        {
            int status = m_impl.SetIzone(slotIdx, iz, timeoutMs);
            return HandleStatus(status);
        }
        /**
         * SRX has two available slots for PID.
         * @param slotIdx one or zero depending on which slot caller wants.
         */
        public int SelectProfileSlot(uint slotIdx)
        {
            m_profile = slotIdx; /* only get two slots for now */
            int status = m_impl.SetProfileSlotSelect(m_profile);
            return HandleStatus(status);
        }
        /**
         * Sets control values for closed loop control.
         *
         * @param p Proportional constant.
         * @param i Integration constant.
         * @param d Differential constant.
         * This function does not modify F-gain.  Considerable passing a zero for f
         * using
         * the four-parameter function.
         */
        public void SetPID(uint slotIdx, float p, float i, float d, uint timeoutMs = 0)
        {
            SetP(slotIdx, p, timeoutMs);
            SetI(slotIdx, i, timeoutMs);
            SetD(slotIdx, d, timeoutMs);
        }
        /**
         * Sets control values for closed loop control.
         *
         * @param p Proportional constant.
         * @param i Integration constant.
         * @param d Differential constant.
         * @param f Feedforward constant.
         */
        public void SetPID(uint slotIdx, float p, float i, float d, float f, uint timeoutMs = 0)
        {
            SetP(slotIdx, p, timeoutMs);
            SetI(slotIdx, i, timeoutMs);
            SetD(slotIdx, d, timeoutMs);
            SetF(slotIdx, f, timeoutMs);
        }
        /**
         * Select the feedback device to use in closed-loop
         */
        public void SetFeedbackDevice(FeedbackDevice feedbackDevice)
        {
            /* save the selection so that future setters/getters know which scalars to apply */
            m_feedbackDevice = feedbackDevice;
            /* pass feedback to actual CAN frame */
            int status = m_impl.SetFeedbackDeviceSelect((int)feedbackDevice);
            HandleStatus(status);
        }
        /**
         * Select the feedback device to use in closed-loop
         */
        public void SetStatusFrameRateMs(StatusFrameRate stateFrame, uint periodMs)
        {
            int status = m_impl.SetStatusFrameRate((uint)stateFrame, periodMs);
            HandleStatus(status);
        }

        /**
         * Get the current proportional constant.
         *
         * @return float proportional constant for current profile.
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public float GetP(uint slotIdx)
        {

            LowLevel_TalonSrx.ParamEnum param = (slotIdx != 0) ? LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_P
                                                   : LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_P;
            // Update the info in m_impl.
            int status = m_impl.RequestParam(param);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float p;
            status = m_impl.GetPgain(slotIdx, out p);
            HandleStatus(status);
            return p;
        }

        /**
         * TODO documentation 
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public float GetI(uint slotIdx)
        {

            LowLevel_TalonSrx.ParamEnum param = (slotIdx != 0) ? LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_I
                                                   : LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_I;
            // Update the info in m_impl.

            int status = m_impl.RequestParam(param);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float i;
            status = m_impl.GetIgain(slotIdx, out i);
            HandleStatus(status);
            return i;
        }

        /**
         * TODO documentation
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public float GetD(uint slotIdx)
        {

            LowLevel_TalonSrx.ParamEnum param = (slotIdx != 0) ? LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_D
                                         : LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_D;
            // Update the info in m_impl.
            int status = m_impl.RequestParam(param);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float d;
            status = m_impl.GetDgain(slotIdx, out d);
            HandleStatus(status);
            return d;
        }
        /**
         *
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public float GetF(uint slotIdx)
        {
            LowLevel_TalonSrx.ParamEnum param = (slotIdx != 0) ? LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_F
                                                   : LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_F;
            // Update the info in m_impl.
            int status = m_impl.RequestParam(param);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
            float f;
            status = m_impl.GetFgain(slotIdx, out f);
            HandleStatus(status);
            return f;
        }
        /**
         * @see SelectProfileSlot to choose between the two sets of gains.
         */
        public int GetIzone(uint slotIdx)
        {
            LowLevel_TalonSrx.ParamEnum param = (slotIdx != 0)
                                             ? LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_IZone
                                             : LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_IZone;
            // Update the info in m_impl.
            int status = m_impl.RequestParam(param);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs);

            int iz;
            status = m_impl.GetIzone(slotIdx, out iz);
            HandleStatus(status);
            return iz;
        }

        /**
         * @return the current setpoint; ie, whatever was last passed to Set().
         */
        public float GetSetpoint() { return m_setPoint; }

        /**
         * Returns the voltage coming in from the battery.
         *
         * @return The input voltage in volts.
         */
        public float GetBusVoltage()
        {
            float voltage;
            int status = m_impl.GetBatteryV(out voltage);
            HandleStatus(status);
            return voltage;
        }

        /**
         * @return The voltage being output by the Talon, in Volts.
         */
        public float GetOutputVoltage()
        {
            int throttle11;
            int status = m_impl.GetAppliedThrottle(out throttle11);
            float voltage = GetBusVoltage() * ((float)(throttle11) / 1023.0f);
            HandleStatus(status);
            return voltage;
        }

        /**
         *  Returns the current going through the Talon, in Amperes.
         */
        public float GetOutputCurrent()
        {
            float current;

            int status = m_impl.GetCurrent(out current);
            HandleStatus(status);

            return current;
        }

        /**
         *  Returns temperature of Talon, in degrees Celsius.
         */
        public float GetTemperature()
        {
            float temp;

            int status = m_impl.GetTemp(out temp);
            HandleStatus(status);
            return temp;
        }
        /**
         * Set the position value of the selected sensor.  This is useful for zero-ing
         * quadrature encoders.
         * Continuous sensors (like analog encoderes) can also partially be set (the
         * portion of the postion based on overflows).
         */
        public int SetPosition(float pos, uint timeoutMs = 0)
        {
            Int32 nativePos = ScaleRotationsToNativeUnits(m_feedbackDevice, pos);
            int status = m_impl.SetSensorPosition(nativePos, timeoutMs);
            return HandleStatus(status);
        }
        /**
         * TODO documentation
         *
         * @return The position of the sensor currently providing feedback.
         *       When using analog sensors, 0 units corresponds to 0V, 1023
         * units corresponds to 3.3V
         *       When using an analog encoder (wrapping around 1023 => 0 is
         * possible) the units are still 3.3V per 1023 units.
         *       When using quadrature, each unit is a quadrature edge (4X)
         * mode.
         */
        public float GetPosition()
        {
            Int32 position;
            int status = m_impl.GetSensorPosition(out position);
            HandleStatus(status);
            return ScaleNativeUnitsToRotations(m_feedbackDevice, position);
        }
        /**
         * If sensor and motor are out of phase, sensor can be inverted
         * (position and velocity multiplied by -1).
         * @see GetPosition and @see GetSpeed.
         */
        public void SetSensorDirection(bool reverseSensor)
        {
            int status = m_impl.SetRevFeedbackSensor(reverseSensor);
            HandleStatus(status);
        }
        /**
         * Flips the sign (multiplies by negative one) the throttle values going into
         * the motor on the talon in closed loop modes.  Typically the application
         * should use SetSensorDirection to keep sensor and motor in phase.
         * @see SetSensorDirection
         * However this routine is helpful for reversing the motor direction
         * when Talon is in slave mode, or when using a single-direction position
         * sensor in a closed-loop mode.
         *
         * @param reverseOutput True if motor output should be flipped; False if not.
         */
        public void SetClosedLoopOutputDirection(bool reverseOutput)
        {
            int status = m_impl.SetRevMotDuringCloseLoopEn(reverseOutput);
            HandleStatus(status);
        }
        /**
         * Returns the current error in the controller.
         *
         * @return the difference between the setpoint and the sensor value.
         */
        public int GetClosedLoopError()
        {
            int error;
            /* retrieve the closed loop error in native units */
            int status = m_impl.GetCloseLoopErr(out error);
            HandleStatus(status);
            return error;
        }
        /**
         * Set the allowable closed loop error.
         * @param allowableCloseLoopError allowable closed loop error for selected profile.
         *       mA for Curent closed loop.
         *       Talon Native Units for position and velocity.
         */
        public void SetAllowableClosedLoopErr(uint slotIdx, UInt32 allowableCloseLoopError, uint timeoutMs = 0)
        {
            /* grab param enum */
            LowLevel_TalonSrx.ParamEnum param;
            if (slotIdx == 1)
            {
                param = LowLevel_TalonSrx.ParamEnum.eProfileParamSlot1_AllowableClosedLoopErr;
            }
            else
            {
                param = LowLevel_TalonSrx.ParamEnum.eProfileParamSlot0_AllowableClosedLoopErr;
            }
            /* send allowable close loop er in native units */
            ConfigSetParameter(param, allowableCloseLoopError, timeoutMs);
        }

        /**
         * TODO documentation
         *
         * @returns The speed of the sensor currently providing feedback.
         *
         * The speed units will be in the sensor's native ticks per 100ms.
         *
         * For analog sensors, 3.3V corresponds to 1023 units.
         *     So a speed of 200 equates to ~0.645 dV per 100ms or 6.451 dV per
         * second.
         *     If this is an analog encoder, that likely means 1.9548 rotations
         * per sec.
         * For quadrature encoders, each unit corresponds a quadrature edge (4X).
         *     So a 250 count encoder will produce 1000 edge events per
         * rotation.
         *     An example speed of 200 would then equate to 20% of a rotation
         * per 100ms,
         *     or 10 rotations per second.
         */
        public float GetSpeed()
        {
            Int32 speed;
            int status = m_impl.GetSensorVelocity(out speed);
            HandleStatus(status);
            return ScaleNativeUnitsToRpm(m_feedbackDevice, speed);
        }

        /**
         * Get the position of whatever is in the analog pin of the Talon, regardless of
         * whether it is actually being used for feedback.
         *
         * @returns The 24bit analog value.  The bottom ten bits is the ADC (0 - 1023)
         *          on the analog pin of the Talon.
         *          The upper 14 bits tracks the overflows and
         * underflows (continuous sensor).
         */
        public int GetAnalogIn()
        {
            int position;
            int status = m_impl.GetAnalogInWithOv(out position);
            HandleStatus(status);
            return position;
        }

        public void SetAnalogPosition(int newPosition)
        {
            int status = m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eAinPosition, newPosition);
            HandleStatus(status);
        }
        /**
         * Get the position of whatever is in the analog pin of the Talon, regardless of
         * whether it is actually being used for feedback.
         *
         * @returns The ADC (0 - 1023) on analog pin of the Talon.
         */
        public int GetAnalogInRaw() { return GetAnalogIn() & 0x3FF; }
        /**
         * Get the position of whatever is in the analog pin of the Talon, regardless of
         * whether it is actually being used for feedback.
         *
         * @returns The value (0 - 1023) on the analog pin of the Talon.
         */
        public int GetAnalogInVel()
        {
            int vel;
            int status = m_impl.GetAnalogInVel(out vel);
            HandleStatus(status);
            return vel;
        }

        /**
         * Get the position of whatever is in the analog pin of the Talon, regardless of
         * whether it is actually being used for feedback.
         *
         * @returns The value (0 - 1023) on the analog pin of the Talon.
         */
        public int GetEncPosition()
        {
            int position;
            int status = m_impl.GetEncPosition(out position);
            HandleStatus(status);
            return position;
        }
        public void SetEncPosition(int newPosition)
        {
            int status = m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eEncPosition, newPosition);
            HandleStatus(status);
        }

        /**
         * Get the position of whatever is in the analog pin of the Talon, regardless of
         * whether it is actually being used for feedback.
         *
         * @returns The value (0 - 1023) on the analog pin of the Talon.
         */
        public int GetEncVel()
        {
            int vel;
            int status = m_impl.GetEncVel(out vel);
            HandleStatus(status);
            return vel;
        }
        public int GetPulseWidthPosition()
        {
            int param;
            int status = m_impl.GetPulseWidthPosition(out param);
            HandleStatus(status);
            return param;
        }
        public void SetPulseWidthPosition(int newPosition)
        {
            int status = m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.ePwdPosition, newPosition);
            HandleStatus(status);
        }
        public int GetPulseWidthVelocity()
        {
            int param;
            int status = m_impl.GetPulseWidthVelocity(out param);
            HandleStatus(status);
            return param;
        }
        public int GetPulseWidthRiseToFallUs()
        {
            int param;
            int status = m_impl.GetPulseWidthRiseToFallUs(out param);
            HandleStatus(status);
            return param;
        }
        public int GetPulseWidthRiseToRiseUs()
        {
            int param;
            int status = m_impl.GetPulseWidthRiseToRiseUs(out param);
            HandleStatus(status);
            return param;
        }
        /**
         * @param which feedback sensor to check it if is connected.
         * @return status of caller's specified sensor type.
         */
        public FeedbackDeviceStatus IsSensorPresent(FeedbackDevice feedbackDevice)
        {
            FeedbackDeviceStatus retval = FeedbackDeviceStatus.FeedbackStatusUnknown;
            int param;
            /* detecting sensor health depends on which sensor caller cares about */
            switch (feedbackDevice)
            {
                case FeedbackDevice.QuadEncoder:
                case FeedbackDevice.AnalogPot:
                case FeedbackDevice.AnalogEncoder:
                case FeedbackDevice.EncRising:
                case FeedbackDevice.EncFalling:
                    /* no real good way to tell if these sensor
                      are actually present so return status unknown. */
                    break;
                case FeedbackDevice.PulseWidth:
                case FeedbackDevice.CtreMagEncoder_Relative:
                case FeedbackDevice.CtreMagEncoder_Absolute:
                    /* all of these require pulse width signal to be present. */
                    int status = m_impl.IsPulseWidthSensorPresent(out param);
                    if (status != CAN_OK)
                    {
                        /* we're not getting status info, signal unknown status */
                    }
                    else
                    {
                        /* param is updated */
                        if (param != 0)
                        {
                            /* pulse signal is present, sensor must be working since it always
                              generates a pulse waveform.*/
                            retval = FeedbackDeviceStatus.FeedbackStatusPresent;
                        }
                        else
                        {
                            /* no pulse present, sensor disconnected */
                            retval = FeedbackDeviceStatus.FeedbackStatusNotPresent;
                        }
                    }
                    break;
            }
            return retval;
        }
        /**
         * @return IO level of QUADA pin.
         */
        public bool GetPinStateQuadA()
        {
            bool retval;
            int status = m_impl.GetQuadApin(out retval);
            HandleStatus(status);
            return retval;
        }
        /**
         * @return IO level of QUADB pin.
         */
        public bool GetPinStateQuadB()
        {
            bool retval;
            int status = m_impl.GetQuadBpin(out retval);
            HandleStatus(status);
            return retval;
        }
        /**
         * @return IO level of QUAD Index pin.
         */
        public bool GetPinStateQuadIdx()
        {
            bool retval;
            int status = m_impl.GetQuadIdxpin(out retval);
            HandleStatus(status);
            return retval;
        }
        /**
         * @return '1' iff forward limit switch is closed, 0 iff switch is open.
         * This function works regardless if limit switch feature is enabled.
         */
        public bool IsFwdLimitSwitchClosed()
        {
            bool retval;
            int status = m_impl.GetLimitSwitchClosedFor(out retval); /* rename this func, '1' => open, '0' => closed */
            HandleStatus(status);
            return retval ? false : true;
        }
        /**
         * @return '1' iff reverse limit switch is closed, 0 iff switch is open.
         * This function works regardless if limit switch feature is enabled.
         */
        public bool IsRevLimitSwitchClosed()
        {
            bool retval;
            int status = m_impl.GetLimitSwitchClosedRev(out retval); /* rename this func, '1' => open, '0' => closed */
            HandleStatus(status);
            return retval ? false : true;
        }
        /**
         * @return '1' iff ClearPosOnIdx is enabled, 0 iff setting is disabled.
         * This function works regardless if limit switch feature is enabled.
         */
        public bool IsZeroSensorPositionOnOdxEnabled()
        {
            bool retval;
            int status = m_impl.GetClearPosOnIdx(out retval);
            HandleStatus(status);
            return retval;
        }
        /**
         * @return '1' iff ClearPosOnLimR is enabled, 0 iff setting is disabled.
         * This function works regardless if limit switch feature is enabled.
         */
        public bool IsZeroSensorPositionOnReverseLimitEnabled()
        {
            bool retval;
            int status = m_impl.GetClearPosOnLimR(out retval);
            HandleStatus(status);
            return retval;
        }
        /**
         * @return '1' iff ClearPosOnLimF is enabled, 0 iff setting is disabled.
         * This function works regardless if limit switch feature is enabled.
         */
        public bool IsZeroSensorPositionOnForwardLimitEnabled()
        {
            bool retval;
            int status = m_impl.GetClearPosOnLimF(out retval);
            HandleStatus(status);
            return retval;
        }
        /*
         * Simple accessor for tracked rise eventso index pin.
         * @return number of rising edges on idx pin.
         */
        public int GetNumberOfQuadIdxRises()
        {
            int rises;
            int status = m_impl.GetEncIndexRiseEvents(out rises); /* rename this func, '1' => open, '0' => closed */
            HandleStatus(status);
            return rises;
        }
        /*
         * @param rises integral value to set into index-rises register.  Great way to
         * zero the index count.
         */
        public void SetNumberOfQuadIdxRises(int rises)
        {
            int status = m_impl.SetParam(
                LowLevel_TalonSrx.ParamEnum.eEncIndexRiseEvents,
                rises); /* rename this func, '1' => open, '0' => closed */
            HandleStatus(status);
        }
        /**
         * TODO documentation
         */
        public bool GetForwardLimitOK()
        {
            bool limSwit;
            bool softLim;
            int status = CAN_OK;
            status = m_impl.GetFault_ForSoftLim(out softLim);
            HandleStatus(status);
            status = m_impl.GetFault_ForLim(out limSwit);
            HandleStatus(status);
            /* If either fault is asserted, signal caller we are disabled (with false?) */
            return (softLim || limSwit) ? false : true;
        }

        /**
         * TODO documentation
         */
        public bool GetReverseLimitOK()
        {
            bool limSwit;
            bool softLim;
            int status = CAN_OK;
            status = m_impl.GetFault_RevSoftLim(out softLim);
            HandleStatus(status);
            status = m_impl.GetFault_RevLim(out limSwit);
            HandleStatus(status);
            /* If either fault is asserted, signal caller we are disabled (with false?) */
            return (softLim || limSwit) ? false : true;
        }

        /**
         * TODO documentation
         */
        public UInt32 GetFaults()
        {
            UInt32 retval = 0;
            bool val;
            int status = CAN_OK;

            /* temperature */
            status = m_impl.GetFault_OverTemp(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kTemperatureFault : 0;

            /* voltage */
            status = m_impl.GetFault_UnderVoltage(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kBusVoltageFault : 0;

            /* fwd-limit-switch */
            status = m_impl.GetFault_ForLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kFwdLimitSwitch : 0;

            /* rev-limit-switch */
            status = m_impl.GetFault_RevLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kRevLimitSwitch : 0;

            /* fwd-soft-limit */
            status = m_impl.GetFault_ForSoftLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kFwdSoftLimit : 0;

            /* rev-soft-limit */
            status = m_impl.GetFault_RevSoftLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kRevSoftLimit : 0;

            return retval;
        }
        public UInt32 GetStickyFaults()
        {
            UInt32 retval = 0;
            bool val;
            int status = CAN_OK;

            /* temperature */
            status = m_impl.GetStckyFault_OverTemp(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kTemperatureFault : 0;

            /* voltage */
            status = m_impl.GetStckyFault_UnderVoltage(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kBusVoltageFault : 0;

            /* fwd-limit-switch */
            status = m_impl.GetStckyFault_ForLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kFwdLimitSwitch : 0;

            /* rev-limit-switch */
            status = m_impl.GetStckyFault_RevLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kRevLimitSwitch : 0;

            /* fwd-soft-limit */
            status = m_impl.GetStckyFault_ForSoftLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kFwdSoftLimit : 0;

            /* rev-soft-limit */
            status = m_impl.GetStckyFault_RevSoftLim(out val);
            HandleStatus(status);
            retval |= (val) ? (UInt32)Faults.kRevSoftLimit : 0;

            return retval;
        }
        public void ClearStickyFaults()
        {
            int status = m_impl.ClearStickyFaults();
            Reporting.SetError(status, Reporting.getHALErrorMessage(status));
        }

        /**
         * Set the maximum voltage change rate.  This ramp rate is in affect regardless
         * of which control mode
         * the TALON is in.
         *
         * When in PercentVbus or Voltage output mode, the rate at which the voltage
         * changes can
         * be limited to reduce current spikes.  Set this to 0.0 to disable rate
         * limiting.
         *
         * @param rampRate The maximum rate of voltage change in Percent Voltage mode in
         * V/s.
         */
        public void SetVoltageRampRate(float rampRate)
        {
            /* Caller is expressing ramp as Voltage per sec, assuming 12V is full.
                    Talon's throttle ramp is in dThrot/d10ms.  1023 is full fwd, -1023 is
               full rev. */
            int rampRatedThrotPer10ms = 0;
            if (rampRate <= 0)
            {
                /* caller wants to disable feature */
            }
            else
            {
                /* desired ramp rate is positive and nonzero */
                rampRatedThrotPer10ms = (int)((rampRate * 1023.0f / 12.0f) / 100f);
                if (rampRatedThrotPer10ms == 0)
                    rampRatedThrotPer10ms = 1; /* slowest ramp possible */
                else if (rampRatedThrotPer10ms > 255)
                    rampRatedThrotPer10ms = 255; /* fastest nonzero ramp */
            }
            int status = m_impl.SetRampThrottle(rampRatedThrotPer10ms);
            HandleStatus(status);
        }
        public void SetVoltageCompensationRampRate(float rampRate)
        {
            /* when in voltage compensation mode, the voltage compensation rate
              directly caps the change in target voltage */
            int status = CAN_OK;
            status = m_impl.SetVoltageCompensationRate(rampRate / 1000);
            HandleStatus(status);
        }
        /**
         * Sets a voltage change rate that applies only when a close loop contorl mode
         * is enabled.
         * This allows close loop specific ramp behavior.
         *
         * @param rampRate The maximum rate of voltage change in Percent Voltage mode in
         * V/s.
         */
        public void SetCloseLoopRampRate(uint slotIdx, float rampRate)
        {
            int closeLoopRampRate = (int)(rampRate * 1023.0f / 12.0f / 1000.0f);
            int status = m_impl.SetCloseLoopRampRate(slotIdx, closeLoopRampRate);
            HandleStatus(status);
        }

        /**
         * @return The version of the firmware running on the Talon
         */
        public UInt32 GetFirmwareVersion()
        {
            int firmwareVersion;
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eFirmVers);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs);
            status =
                  m_impl.GetParamResponseInt32(LowLevel_TalonSrx.ParamEnum.eFirmVers, out firmwareVersion);
            HandleStatus(status);

            return (UInt32)firmwareVersion;
        }
        /**
         * @return The accumulator for I gain.
         */
        public int GetIaccum()
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.ePidIaccum);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
            int iaccum;
            status = m_impl.GetParamResponseInt32(LowLevel_TalonSrx.ParamEnum.ePidIaccum, out iaccum);
            HandleStatus(status);
            return iaccum;
        }
        /**
         * Clear the accumulator for I gain.
         */
        public void ClearIaccum()
        {
            int status = m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.ePidIaccum, 0);
            HandleStatus(status);
        }


        /**
         * TODO documentation
         */
        public int ConfigNeutralMode(NeutralMode mode)
        {
            int status = CAN_OK;
            switch (mode)
            {
                default:
                case NeutralMode.Jumper: /* use default setting in flash based on webdash/BrakeCal button selection */
                    status = m_impl.SetOverrideBrakeType(kBrakeOverride_UseDefaultsFromFlash);
                    break;
                case NeutralMode.Brake:
                    status = m_impl.SetOverrideBrakeType(kBrakeOverride_OverrideBrake);
                    break;
                case NeutralMode.Coast:
                    status = m_impl.SetOverrideBrakeType(kBrakeOverride_OverrideCoast);
                    break;
            }
            return HandleStatus(status);
        }
        /**
         * @return nonzero if brake is enabled during neutral.  Zero if coast is enabled
         * during neutral.
         */
        public bool GetBrakeEnableDuringNeutral()
        {
            bool brakeEn = false;
            int status = m_impl.GetBrakeIsEnabled(out brakeEn);
            HandleStatus(status);
            return brakeEn;
        }
        /**
         * Configure how many codes per revolution are generated by your encoder.
         *
         * @param codesPerRev The number of counts per revolution.
         */
        public void ConfigEncoderCodesPerRev(UInt16 codesPerRev)
        {
            /* first save the scalar so that all getters/setter work as the user expects */
            m_codesPerRev = codesPerRev;
            /* next send the scalar to the Talon over CAN.  This is so that the Talon can report
              it to whoever needs it, like the webdash.  Don't bother checking the return,
              this is only for instrumentation and is not necessary for Talon functionality. */
            m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eNumberEncoderCPR, m_codesPerRev);
        }

        /**
         * Configure the number of turns on the potentiometer.
         *
         * @param turns The number of turns of the potentiometer.
         */
        public void ConfigPotentiometerTurns(UInt16 turns)
        {
            /* first save the scalar so that all getters/setter work as the user expects */
            m_numPotTurns = turns;
            /* next send the scalar to the Talon over CAN.  This is so that the Talon can report
              it to whoever needs it, like the webdash.  Don't bother checking the return,
              this is only for instrumentation and is not necessary for Talon functionality. */
            m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eNumberPotTurns, m_numPotTurns);
        }

        /**
         * @deprecated not implemented
         */
        public void ConfigSoftPositionLimits(float forwardLimitPosition,
                                                float reverseLimitPosition, uint timeoutMs = 0)
        {
            ConfigLimitMode(LimitMode.kLimitMode_SoftPositionLimits);
            ConfigForwardLimit(forwardLimitPosition, timeoutMs);
            ConfigReverseLimit(reverseLimitPosition, timeoutMs);
        }

        /**
         * TODO documentation
         */
        public void DisableSoftPositionLimits()
        {
            ConfigLimitMode(LimitMode.kLimitMode_SwitchInputsOnly);
        }

        /**
         * TODO documentation
         * Configures the soft limit enable (wear leveled persistent memory).
         * Also sets the limit switch overrides.
         */
        public void ConfigLimitMode(LimitMode mode)
        {
            int status = CAN_OK;
            switch (mode)
            {
                case LimitMode.kLimitMode_SwitchInputsOnly: /** Only use switches for limits */
                    /* turn OFF both limits. SRX has individual enables and polarity for each
                     * limit switch.*/
                    status = m_impl.SetForwardSoftEnable(0);
                    HandleStatus(status);
                    status = m_impl.SetReverseSoftEnable(0);
                    HandleStatus(status);
                    {
                        Reporting.SetError(status, Reporting.getHALErrorMessage(status));
                    }
                    /* override enable the limit switches, this circumvents the webdash */
                    status = m_impl.SetOverrideLimitSwitchEn(
                        kLimitSwitchOverride_EnableFwd_EnableRev);
                    HandleStatus(status);
                    break;
                case LimitMode.kLimitMode_SoftPositionLimits: /** Use both switches and soft limits */
                    /* turn on both limits. SRX has individual enables and polarity for each
                     * limit switch.*/
                    status = m_impl.SetForwardSoftEnable(1);
                    HandleStatus(status);
                    status = m_impl.SetReverseSoftEnable(1);
                    HandleStatus(status);
                    /* override enable the limit switches, this circumvents the webdash */
                    status = m_impl.SetOverrideLimitSwitchEn(kLimitSwitchOverride_EnableFwd_EnableRev);
                    HandleStatus(status);
                    break;

                case LimitMode.kLimitMode_SrxDisableSwitchInputs: /** disable both limit switches and
                                               soft limits */
                    /* turn on both limits. SRX has individual enables and polarity for each
                     * limit switch.*/
                    status = m_impl.SetForwardSoftEnable(0);
                    HandleStatus(status);
                    status = m_impl.SetReverseSoftEnable(0);
                    HandleStatus(status);
                    /* override enable the limit switches, this circumvents the webdash */
                    status = m_impl.SetOverrideLimitSwitchEn(kLimitSwitchOverride_DisableFwd_DisableRev);
                    HandleStatus(status);
                    break;
            }
        }

        /**
         * TODO documentation
         */
        public void ConfigForwardLimit(float forwardLimitPosition, uint timeoutMs = 0)
        {
            int status = CAN_OK;
            Int32 nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, forwardLimitPosition);
            status = m_impl.SetForwardSoftLimit(nativeLimitPos, timeoutMs);
            HandleStatus(status);
        }
        /**
         * Change the fwd limit switch setting to normally open or closed.
         * Talon will disable momentarilly if the Talon's current setting
         * is dissimilar to the caller's requested setting.
         *
         * Since Talon saves setting to flash this should only affect
         * a given Talon initially during robot install.
         *
         * @param normallyOpen true for normally open.  false for normally closed.
         */
        public void ConfigFwdLimitSwitchNormallyOpen(bool normallyOpen)
        {
            int status =
                m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eOnBoot_LimitSwitch_Forward_NormallyClosed,
                                 normallyOpen ? 0 : 1);
            HandleStatus(status);
        }
        /**
         * Change the rev limit switch setting to normally open or closed.
         * Talon will disable momentarilly if the Talon's current setting
         * is dissimilar to the caller's requested setting.
         *
         * Since Talon saves setting to flash this should only affect
         * a given Talon initially during robot install.
         *
         * @param normallyOpen true for normally open.  false for normally closed.
         */
        public void ConfigRevLimitSwitchNormallyOpen(bool normallyOpen)
        {
            int status =
                m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eOnBoot_LimitSwitch_Reverse_NormallyClosed,
                                 normallyOpen ? 0 : 1);
            HandleStatus(status);
        }
        /**
         * TODO documentation
         */
        public void ConfigReverseLimit(float reverseLimitPosition, uint timeoutMs = 0)
        {
            int status = CAN_OK;
            Int32 nativeLimitPos = ScaleRotationsToNativeUnits(m_feedbackDevice, reverseLimitPosition);
            status = m_impl.SetReverseSoftLimit(nativeLimitPos, timeoutMs);
            HandleStatus(status);
        }
        /**
         * TODO documentation
         */
        public void ConfigMaxOutputVoltage(float voltage, uint timeoutMs = 0)
        {
            /* config peak throttle when in closed-loop mode in the fwd and rev direction. */
            ConfigPeakOutputVoltage(voltage, -voltage, timeoutMs);
        }
        public int ConfigPeakOutputVoltage(float forwardVoltage, float reverseVoltage, uint timeoutMs = 0)
        {
            int status1 = CAN_OK, status2 = CAN_OK;
            /* bounds checking */
            if (forwardVoltage > 12)
                forwardVoltage = 12;
            else if (forwardVoltage < 0)
                forwardVoltage = 0;
            if (reverseVoltage > 0)
                reverseVoltage = 0;
            else if (reverseVoltage < -12)
                reverseVoltage = -12;
            /* config calls */
            status1 = ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.ePeakPosOutput, 1023f * forwardVoltage / 12.0f, timeoutMs);
            status2 = ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.ePeakNegOutput, 1023f * reverseVoltage / 12.0f, timeoutMs);
            /* return the worst one */
            if (status1 == CAN_OK)
                status1 = status2;
            return status1;
        }
        public int ConfigNominalOutputVoltage(float forwardVoltage, float reverseVoltage, uint timeoutMs = 0)
        {
            int status1 = CAN_OK, status2 = CAN_OK;
            /* bounds checking */
            if (forwardVoltage > 12)
                forwardVoltage = 12;
            else if (forwardVoltage < 0)
                forwardVoltage = 0;
            if (reverseVoltage > 0)
                reverseVoltage = 0;
            else if (reverseVoltage < -12)
                reverseVoltage = -12;
            /* config calls */
            status1 = ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eNominalPosOutput, 1023f * forwardVoltage / 12.0f, timeoutMs);
            status2 = ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eNominalNegOutput, 1023f * reverseVoltage / 12.0f, timeoutMs);
            /* return the worst one */
            if (status1 == CAN_OK)
                status1 = status2;
            return status1;
        }
        /**
         * General set frame.  Since the parameter is a general integral type, this can
         * be used for testing future features.
         */
        public int ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum paramEnum, float value, uint timeoutMs = 0)
        {
            int status;
            /* config peak throttle when in closed-loop mode in the positive direction. */
            status = m_impl.SetParam(paramEnum, value, timeoutMs);
            return HandleStatus(status);
        }
        public void ConfigSetParameter(UInt32 paramEnum, int value, uint timeoutMs = 0)
        {
            int status;
            /* config peak throttle when in closed-loop mode in the positive direction. */
            status = m_impl.SetParam((LowLevel_TalonSrx.ParamEnum)paramEnum, value, timeoutMs);
            HandleStatus(status);
        }
        /**
         * General get frame.  Since the parameter is a general integral type, this can
         * be used for testing future features.
         */
        public bool GetParameter(UInt32 paramEnum, out float dvalue)
        {
            bool retval = true;
            /* send the request frame */
            int status = m_impl.RequestParam((LowLevel_TalonSrx.ParamEnum)paramEnum);
            HandleStatus(status);
            if (status != CAN_OK)
            {
                retval = false;
            }
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
                                                                        /* get the last received update */
            status = m_impl.GetParamResponse((LowLevel_TalonSrx.ParamEnum)paramEnum, out dvalue);
            HandleStatus(status);
            if (status != CAN_OK)
            {
                retval = false;
            }
            return retval;
        }


        /**
         * Fixup the sendMode so Set() serializes the correct demand value.
         * Also fills the modeSelecet in the control frame to disabled.
         * @param mode Control mode to ultimately enter once user calls Set().
         * @see Set()
         */
        private void ApplyControlMode(ControlMode mode)
        {
            m_controlMode = mode;

            switch (mode)
            {
                case ControlMode.kPercentVbus:
                    m_sendMode = TalonControlMode.kThrottle;
                    break;
                case ControlMode.kCurrent:
                    m_sendMode = TalonControlMode.kCurrentMode;
                    break;
                case ControlMode.kSpeed:
                    m_sendMode = TalonControlMode.kSpeedMode;
                    break;
                case ControlMode.kPosition:
                    m_sendMode = TalonControlMode.kPositionMode;
                    break;
                case ControlMode.kVoltage:
                    m_sendMode = TalonControlMode.kVoltageMode;
                    break;
                case ControlMode.kFollower:
                    m_sendMode = TalonControlMode.kFollowerMode;
                    break;
                case ControlMode.kMotionProfile:
                    m_sendMode = TalonControlMode.kMotionProfileMode;
                    break;
                case ControlMode.kMotionMagic:
                    m_sendMode = TalonControlMode.kMotionMagic;
                    break;
            }
            // Keep the talon disabled until Set() is called.
            int status = m_impl.SetModeSelect((int)TalonControlMode.kDisabled);
            HandleStatus(status);
        }
        /**
         * TODO documentation
         */
        public void SetControlMode(ControlMode mode)
        {
            if (m_controlMode == mode)
            {
                /* we already are in this mode, don't perform disable workaround */
            }
            else
            {
                ApplyControlMode(mode);
            }
        }

        /**
         * TODO documentation
         */
        public ControlMode GetControlMode()
        {
            return m_controlMode;
        }

        /**
         * @param devToLookup FeedbackDevice to lookup the scalar for.  Because Talon
         *            allows multiple sensors to be attached simultaneously, caller must
         *            specify which sensor to lookup.
         * @return    The number of native Talon units per rotation of the selected sensor.
         *            Zero if the necessary sensor information is not available.
         * @see ConfigEncoderCodesPerRev
         * @see ConfigPotentiometerTurns
         */
        private float GetNativeUnitsPerRotationScalar(FeedbackDevice devToLookup)
        {
            bool scalingAvail = false;
            int status = CAN_OK;
            float retval = 0;
            switch (devToLookup)
            {
                case FeedbackDevice.QuadEncoder:
                    { /* When caller wants to lookup Quadrature, the QEI may be in 1x if the selected feedback is edge counter.
       * Additionally if the quadrature source is the CTRE Mag encoder, then the CPR is known.
       * This is nice in that the calling app does not require knowing the CPR at all.
       * So do both checks here.
       */
                        Int32 qeiPulsePerCount = 4; /* default to 4x */
                        switch (m_feedbackDevice)
                        {
                            case FeedbackDevice.CtreMagEncoder_Relative:
                            case FeedbackDevice.CtreMagEncoder_Absolute:
                                /* we assume the quadrature signal comes from the MagEnc,
                                  of which we know the CPR already */
                                retval = kNativePwdUnitsPerRotation;
                                scalingAvail = true;
                                break;
                            case FeedbackDevice.EncRising: /* Talon's QEI is setup for 1x, so perform 1x math */
                            case FeedbackDevice.EncFalling:
                                qeiPulsePerCount = 1;
                                break;
                            case FeedbackDevice.QuadEncoder: /* Talon's QEI is 4x */
                            default: /* pulse width and everything else, assume its regular quad use. */
                                break;
                        }
                        if (scalingAvail)
                        {
                            /* already deduced the scalar above, we're done. */
                        }
                        else
                        {
                            /* we couldn't deduce the scalar just based on the selection */
                            if (0 == m_codesPerRev)
                            {
                                /* caller has never set the CPR.  Most likely caller
                                  is just using engineering units so fall to the
                                  bottom of this func.*/
                            }
                            else
                            {
                                /* Talon expects PPR units */
                                retval = qeiPulsePerCount * m_codesPerRev;
                                scalingAvail = true;
                            }
                        }
                    }
                    break;
                case FeedbackDevice.EncRising:
                case FeedbackDevice.EncFalling:
                    if (0 == m_codesPerRev)
                    {
                        /* caller has never set the CPR.  Most likely caller
                          is just using engineering units so fall to the
                          bottom of this func.*/
                    }
                    else
                    {
                        /* Talon expects PPR units */
                        retval = 1 * m_codesPerRev;
                        scalingAvail = true;
                    }
                    break;
                case FeedbackDevice.AnalogPot:
                case FeedbackDevice.AnalogEncoder:
                    if (0 == m_numPotTurns)
                    {
                        /* caller has never set the CPR.  Most likely caller
                          is just using engineering units so fall to the
                          bottom of this func.*/
                    }
                    else
                    {
                        retval = (float)kNativeAdcUnitsPerRotation / m_numPotTurns;
                        scalingAvail = true;
                    }
                    break;
                case FeedbackDevice.CtreMagEncoder_Relative:
                case FeedbackDevice.CtreMagEncoder_Absolute:
                case FeedbackDevice.PulseWidth:
                    retval = kNativePwdUnitsPerRotation;
                    scalingAvail = true;
                    break;
            }
            /* handle any detected errors */
            HandleStatus(status);
            /* if scaling information is not possible, signal caller
              by returning zero */
            if (false == scalingAvail)
                retval = 0;
            return retval;
        }
        /**
         * @param fullRotations   float precision value representing number of rotations of selected feedback sensor.
         *              If user has never called the config routine for the selected sensor, then the caller
         *              is likely passing rotations in engineering units already, in which case it is returned
         *              as is.
         *              @see ConfigPotentiometerTurns
         *              @see ConfigEncoderCodesPerRev
         * @return fullRotations in native engineering units of the Talon SRX firmware.
         */
        private Int32 ScaleRotationsToNativeUnits(FeedbackDevice devToLookup, float fullRotations)
        {
            /* first assume we don't have config info, prep the default return */
            Int32 retval = (Int32)fullRotations;
            /* retrieve scaling info */
            float scalar = GetNativeUnitsPerRotationScalar(devToLookup);
            /* apply scalar if its available */
            if (scalar > 0)
                retval = (Int32)(fullRotations * scalar);
            return retval;
        }
        /**
         * @param rpm   float precision value representing number of rotations per minute of selected feedback sensor.
         *              If user has never called the config routine for the selected sensor, then the caller
         *              is likely passing rotations in engineering units already, in which case it is returned
         *              as is.
         *              @see ConfigPotentiometerTurns
         *              @see ConfigEncoderCodesPerRev
         * @return sensor velocity in native engineering units of the Talon SRX firmware.
         */
        private Int32 ScaleVelocityToNativeUnits(FeedbackDevice devToLookup, float rpm)
        {
            /* first assume we don't have config info, prep the default return */
            Int32 retval = (Int32)rpm;
            /* retrieve scaling info */
            float scalar = GetNativeUnitsPerRotationScalar(devToLookup);
            /* apply scalar if its available */
            if (scalar > 0)
                retval = (Int32)(rpm * kMinutesPer100msUnit * scalar);
            return retval;
        }
        /**
         * @param nativePos   integral position of the feedback sensor in native Talon SRX units.
         *              If user has never called the config routine for the selected sensor, then the return
         *              will be in TALON SRX units as well to match the behavior in the 2015 season.
         *              @see ConfigPotentiometerTurns
         *              @see ConfigEncoderCodesPerRev
         * @return float precision number of rotations, unless config was never performed.
         */
        private float ScaleNativeUnitsToRotations(FeedbackDevice devToLookup, Int32 nativePos)
        {
            /* first assume we don't have config info, prep the default return */
            float retval = (float)nativePos;
            /* retrieve scaling info */
            float scalar = GetNativeUnitsPerRotationScalar(devToLookup);
            /* apply scalar if its available */
            if (scalar > 0)
                retval = ((float)nativePos) / scalar;
            return retval;
        }
        /**
         * @param nativeVel   integral velocity of the feedback sensor in native Talon SRX units.
         *              If user has never called the config routine for the selected sensor, then the return
         *              will be in TALON SRX units as well to match the behavior in the 2015 season.
         *              @see ConfigPotentiometerTurns
         *              @see ConfigEncoderCodesPerRev
         * @return float precision of sensor velocity in RPM, unless config was never performed.
         */
        private float ScaleNativeUnitsToRpm(FeedbackDevice devToLookup, Int32 nativeVel)
        {
            /* first assume we don't have config info, prep the default return */
            float retval = (float)nativeVel;
            /* retrieve scaling info */
            float scalar = GetNativeUnitsPerRotationScalar(devToLookup);
            /* apply scalar if its available */
            if (scalar > 0)
                retval = (float)(nativeVel) / (scalar * kMinutesPer100msUnit);
            return retval;
        }

        /**
         * Enables Talon SRX to automatically zero the Sensor Position whenever an
         * edge is detected on the index signal.
         * @param enable     boolean input, pass true to enable feature or false to disable.
         * @param risingEdge   boolean input, pass true to clear the position on rising edge,
         *          pass false to clear the position on falling edge.
         */
        public void EnableZeroSensorPositionOnIndex(bool enable, bool risingEdge, uint timeoutMs = 0)
        {
            if (enable)
            {
                /* enable the feature, update the edge polarity first to ensure
                  it is correct before the feature is enabled. */
                ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eQuadIdxPolarity, risingEdge ? 1 : 0, timeoutMs);
                ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eClearPositionOnIdx, 1, timeoutMs);
            }
            else
            {
                /* disable the feature first, then update the edge polarity. */
                ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eClearPositionOnIdx, 0, timeoutMs);
                ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eQuadIdxPolarity, risingEdge ? 1 : 0, timeoutMs);
            }
        }
        /**
         * Enables Talon SRX to automatically zero the Sensor Position whenever an
         * edge is detected on the Forward Limit Switch signal.
         * @param enable     boolean input, pass true to enable feature or false to disable.
         */
		 public void EnableZeroSensorPositionOnForwardLimit(bool enable, uint timeoutMs = 0)
		 {
			 ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eClearPositionOnLimitF, enable ? 1 : 0, timeoutMs);
		 }
		 /**
         * Enables Talon SRX to automatically zero the Sensor Position whenever an
         * edge is detected on the Reverse Limit Switch signal.
         * @param enable     boolean input, pass true to enable feature or false to disable.
         */
		 public void EnableZeroSensorPositionOnReverseLimit(bool enable, uint timeoutMs = 0)
		 {
			 ConfigSetParameter(CTRE.LowLevel_TalonSrx.ParamEnum.eClearPositionOnLimitR, enable ? 1 : 0, timeoutMs);
        }
        /**
         * @param voltage       Motor voltage to output when closed loop features are being used (Position,
         *                      Speed, Motion Profile, Motion Magic, etc.) in volts.
         *                      Pass 0 to disable feature.  Input should be within [0.0 V,255.0 V]
         * @param timeoutMs     Optional integer input for a blocking timeout to wait for transmit.
         */
        public void SetNominalClosedLoopVoltage(float voltage, uint timeoutMs = 0)
        {
            m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eNominalBatteryVoltage, voltage, timeoutMs);
        }
        /**
         * Disables the nominal closed loop voltage compensation.
         * Same as calling SetNominalClosedLoopVoltage(0).
         * @param timeoutMs     Optional integer input for a blocking timeout to wait for transmit.
         */
        public void DisableNominalClosedLoopVoltage(uint timeoutMs = 0)
        {
            SetNominalClosedLoopVoltage(0f, timeoutMs);
        }
        /**
         * @return the currently selected nominal closed loop voltage. Zero (Default) means feature is disabled.
         */
        public float GetNominalClosedLoopVoltage()
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eNominalBatteryVoltage);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float retval;
            status = m_impl.GetParamResponse(LowLevel_TalonSrx.ParamEnum.eNominalBatteryVoltage, out retval);
            HandleStatus(status);

            return retval;
        }

        public enum VelocityMeasurementPeriod
        {
            Period_1Ms = 1,
            Period_2Ms = 2,
            Period_5Ms = 5,
            Period_10Ms = 10,
            Period_20Ms = 20,
            Period_25Ms = 25,
            Period_50Ms = 50,
            Period_100Ms = 100,
        };
        /**
         * Sets the duration of time that the Talon measures for each velocity measurement (which occures at each 1ms process loop).
         * The default value is 100, which means that every process loop (1ms), the Talon will measure the change in position
         * between now and 100ms ago, and will insert into a rolling average.
         *
         * Decreasing this from the default (100ms) will yield a less-resolute measurement since there is less time for the sensor to change.
         * This will be perceived as increased granularity in the measurement (or stair-stepping).  But doing so will also decrease the latency 
         * between sensor motion and measurement.
         * 
         * Regardles of this setting value, native velocity units are still in change-in-sensor-per-100ms.
         * 
         * @param period      Support period enum.  Curent valid values are 1,2,5,10,20,25,50, or 100ms.
         * @param timeoutMs     Optional integer input for a blocking timeout to wait for transmit.
         */
        public void SetVelocityMeasurementPeriod(VelocityMeasurementPeriod period, uint timeoutMs = 0)
        {
            m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eSampleVelocityPeriod, (int)period, timeoutMs);
        }
        /**
         * Sets the window size of the rolling average used in velocity measurement.
         * The default value is 64, which means that every process loop (1ms), the Talon will insert a velocity measurement 
         * into a windowed averager with a history of 64 samples.
         * Each sample is inserted every 1ms regardless of what Period is selected. 
         * As a result the window is practically in ms units.
         * 
         * @param windowSize    Window size of rolling average.
         * @param timeoutMs     Optional integer input for a blocking timeout to wait for transmit.
         */
        public void SetVelocityMeasurementWindow(UInt32 windowSize, uint timeoutMs = 0)
        {
            m_impl.SetParam(LowLevel_TalonSrx.ParamEnum.eSampleVelocityWindow, (int)windowSize, timeoutMs);
        }

        public int GetVelocityMeasurementPeriod(out VelocityMeasurementPeriod period)
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eSampleVelocityPeriod);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            int rawValue;
            status = m_impl.GetParamResponseInt32(LowLevel_TalonSrx.ParamEnum.eSampleVelocityPeriod, out rawValue);
            HandleStatus(status);

            period = (VelocityMeasurementPeriod)rawValue;

            if (status == StatusCodes.OK)
            {
                switch (period)
                {
                    case VelocityMeasurementPeriod.Period_1Ms:
                    case VelocityMeasurementPeriod.Period_2Ms:
                    case VelocityMeasurementPeriod.Period_5Ms:
                    case VelocityMeasurementPeriod.Period_10Ms:
                    case VelocityMeasurementPeriod.Period_20Ms:
                    case VelocityMeasurementPeriod.Period_25Ms:
                    case VelocityMeasurementPeriod.Period_50Ms:
                    case VelocityMeasurementPeriod.Period_100Ms:
                        break;
                    default:
                        status = StatusCodes.CAN_INVALID_PARAM;
                        break;
                }
            }
            return status;
        }

        public int GetVelocityMeasurementWindow(out UInt32 window)
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eSampleVelocityPeriod);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            int rawValue;
            status = m_impl.GetParamResponseInt32(LowLevel_TalonSrx.ParamEnum.eSampleVelocityWindow, out rawValue);
            HandleStatus(status);
            window = (UInt32)rawValue;

            return status;
        }

        /**
         * Calling application can opt to speed up the handshaking between the robot API and the Talon to increase the
         * download rate of the Talon's Motion Profile.  Ideally the period should be no more than half the period
         * of a trajectory point.
         */
        public void ChangeMotionControlFramePeriod(uint periodMs)
        {
            m_impl.ChangeMotionControlFramePeriod(periodMs);
        }

        /**
         * Clear the buffered motion profile in both Talon RAM (bottom), and in the API (top).
         * Be sure to check GetMotionProfileStatus() to know when the buffer is actually cleared.
         */
        public void ClearMotionProfileTrajectories()
        {
            m_impl.ClearMotionProfileTrajectories();
        }

        /**
         * Retrieve just the buffer count for the api-level (top) buffer.
         * This routine performs no CAN or data structure lookups, so its fast and ideal
         * if caller needs to quickly poll the progress of trajectory points being emptied
         * into Talon's RAM. Otherwise just use GetMotionProfileStatus.
         * @return number of trajectory points in the top buffer.
         */
        public uint GetMotionProfileTopLevelBufferCount()
        {
            return m_impl.GetMotionProfileTopLevelBufferCount();
        }

        /**
         * Push another trajectory point into the top level buffer (which is emptied into
         * the Talon's bottom buffer as room allows).
         * @param trajPt the trajectory point to insert into buffer.
         * @return true  if trajectory point push ok. CTR_BufferFull if buffer is full
         * due to kMotionProfileTopBufferCapacity.
         */
        public bool PushMotionProfileTrajectory(TrajectoryPoint trajPt)
        {
            /* convert positiona and velocity to native units */
            Int32 targPos = ScaleRotationsToNativeUnits(m_feedbackDevice, trajPt.position);
            Int32 targVel = ScaleVelocityToNativeUnits(m_feedbackDevice, trajPt.velocity);
            /* bounds check signals that require it */
            int profileSlotSelect = (trajPt.profileSlotSelect > 0) ? 1 : 0;
            byte timeDurMs = 255;
            if(trajPt.timeDurMs < 255)
                timeDurMs = (byte)trajPt.timeDurMs; /* cap time to 255ms */
                                                                                 /* send it to the top level buffer */
            int status = m_impl.PushMotionProfileTrajectory(    (int)targPos,
                                                                (int)targVel,
                                                                (int)profileSlotSelect,
                                                                (int)timeDurMs,
                                                                trajPt.velocityOnly ? 1:0,
                                                                trajPt.isLastPoint ? 1 : 0,
                                                                trajPt.zeroPos ? 1 : 0);
            return (status == CAN_OK) ? true : false;
        }
        /**
         * @return true if api-level (top) buffer is full.
         */
        public bool IsMotionProfileTopLevelBufferFull()
        {
            return m_impl.IsMotionProfileTopLevelBufferFull();
        }

        /**
         * This must be called periodically to funnel the trajectory points from the API's top level buffer to
         * the Talon's bottom level buffer.  Recommendation is to call this twice as fast as the executation rate of the motion profile.
         * So if MP is running with 20ms trajectory points, try calling this routine every 10ms.  All motion profile functions are thread-safe
         * through the use of a mutex, so there is no harm in having the caller utilize threading.
         */
        public void ProcessMotionProfileBuffer()
        {
            m_impl.ProcessMotionProfileBuffer();
        }

        /**
         * Retrieve all status information.
         * Since this all comes from one CAN frame, its ideal to have one routine to retrieve the frame once and decode everything.
         * @param [out] motionProfileStatus contains all progress information on the currently running MP.
         */
        public void GetMotionProfileStatus(out MotionProfileStatus motionProfileStatus)
        {
            UInt32 flags;
            UInt32 profileSlotSelect;
            Int32 targPos, targVel;
            UInt32 topBufferRem, topBufferCnt, btmBufferCnt;
            UInt32 outputEnable;
            /* retrieve all motion profile signals from status frame */
            int status = m_impl.GetMotionProfileStatus(out flags, out profileSlotSelect, out targPos, out targVel, out topBufferRem, out topBufferCnt, out btmBufferCnt, out outputEnable);
            /* completely update the caller's structure */
            motionProfileStatus.topBufferRem = topBufferRem;
            motionProfileStatus.topBufferCnt = topBufferCnt;
            motionProfileStatus.btmBufferCnt = btmBufferCnt;
            motionProfileStatus.hasUnderrun = (flags & kMotionProfileFlag_HasUnderrun) > 0;
            motionProfileStatus.isUnderrun = (flags & kMotionProfileFlag_IsUnderrun) > 0;
            motionProfileStatus.activePointValid = (flags & kMotionProfileFlag_ActTraj_IsValid) > 0;
            motionProfileStatus.activePoint.isLastPoint = (flags & kMotionProfileFlag_ActTraj_IsLast) > 0;
            motionProfileStatus.activePoint.velocityOnly = (flags & kMotionProfileFlag_ActTraj_VelOnly) > 0;
            motionProfileStatus.activePoint.position = ScaleNativeUnitsToRotations(m_feedbackDevice, targPos);
            motionProfileStatus.activePoint.velocity = ScaleNativeUnitsToRpm(m_feedbackDevice, targVel);
            motionProfileStatus.activePoint.profileSlotSelect = profileSlotSelect;
            switch (outputEnable)
            {
                case 0:
                    motionProfileStatus.outputEnable = SetValueMotionProfile.Disable;
                    break;
                case 1:
                    motionProfileStatus.outputEnable = SetValueMotionProfile.Enable;
                    break;
                case 2:
                    motionProfileStatus.outputEnable = SetValueMotionProfile.Hold;
                    break;
                default:
                    motionProfileStatus.outputEnable = SetValueMotionProfile.Disable;
                    break;
            }
            motionProfileStatus.activePoint.zeroPos = false; /* this signal is only used sending pts to Talon */
            motionProfileStatus.activePoint.timeDurMs = 0;   /* this signal is only used sending pts to Talon */

            HandleStatus(status);
        }
        /**
         * Clear the hasUnderrun flag in Talon's Motion Profile Executer when MPE is ready for another point,
         * but the low level buffer is empty.
         *
         * Once the Motion Profile Executer sets the hasUnderrun flag, it stays set until
         * Robot Application clears it with this routine, which ensures Robot Application
         * gets a chance to instrument or react.  Caller could also check the isUnderrun flag
         * which automatically clears when fault condition is removed.
         */
        public void ClearMotionProfileHasUnderrun(uint timeoutMs = 0)
        {
            ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eMotionProfileHasUnderrunErr, 0, timeoutMs);
        }
        /**
         * Set the Cruise Velocity used in Motion Magic Control Mode.
         * @param motmagicCruiseVeloc Cruise(peak) velocity in RPM.
         */
        public int SetMotionMagicCruiseVelocity(float motMagicCruiseVeloc, uint timeoutMs = 0)
        {
            int velNative = ScaleVelocityToNativeUnits(m_feedbackDevice, motMagicCruiseVeloc);
            return ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eMotMag_VelCruise, velNative, timeoutMs);
        }
        /**
         * Set the Acceleration used in Motion Magic Control Mode.
         * @param motMagicAccel Accerleration in RPM per second.
         */
        public int SetMotionMagicAcceleration(float motMagicAccel, uint timeoutMs = 0)
        {
            int accel = ScaleVelocityToNativeUnits(m_feedbackDevice, motMagicAccel);
            return ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eMotMag_Accel, accel, timeoutMs);
        }
        /**
         * @return polled motion magic cruise velocity setting from Talon.  
		 * RPM if units are configured, velocity native units otherwise.
         */
        public float GetMotionMagicCruiseVelocity()
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eMotMag_VelCruise);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float retval;
            status = m_impl.GetParamResponse(LowLevel_TalonSrx.ParamEnum.eMotMag_VelCruise, out retval);
            HandleStatus(status);

            return ScaleNativeUnitsToRpm(m_feedbackDevice, (Int32)retval);
        }
        /**
         * @return polled motion magic acceleration setting from Talon.  
		 * RPM per second if units are configured, velocity native units per second otherwise.
         */
        public float GetMotionMagicAcceleration()
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eMotMag_Accel);
            HandleStatus(status);

            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */

            float retval;
            status = m_impl.GetParamResponse(LowLevel_TalonSrx.ParamEnum.eMotMag_Accel, out retval);
            HandleStatus(status);

            return ScaleNativeUnitsToRpm(m_feedbackDevice, (Int32)retval);
        }

        public int SetCurrentLimit(uint amps, uint timeoutMs = 0)
        {
            return ConfigSetParameter(LowLevel_TalonSrx.ParamEnum.eCurrentLimThreshold, amps, timeoutMs);
        }
        public int EnableCurrentLimit(bool enable)
        {
            return m_impl.SetCurrentLimEnable(enable);
        }
        public int SetDataPortOutputPeriod(uint periodMs)
        {
            int status = m_impl.SetDataPortOutputPeriodMs(periodMs);
            HandleStatus(status);
            return status;
        }
        public int SetDataPortOutputEnable(int idx, bool enable)
        {
            int status = m_impl.SetDataPortOutputEnable(idx, enable);
            HandleStatus(status);
            return status;
        }
        public int SetDataPortOutput(int idx, int onTimeMs)
        {
            int status = m_impl.SetDataPortOutputOnTimeMs(idx, onTimeMs);
            HandleStatus(status);
            return status;
        }
        /**
         * @return true iff a reset has occured since last call.
         */
        public bool HasResetOccured()
        {
            return m_impl.HasResetOccured();
        }
        public int GetCustomParam0(out Int32 value)
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eCustomParam0);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
            status = m_impl.GetParamResponseRaw(LowLevel_TalonSrx.ParamEnum.eCustomParam0, out value);
            return HandleStatus(status);
        }
        public int GetCustomParam1(out Int32 value)
        {
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.eCustomParam1);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
            status = m_impl.GetParamResponseRaw(LowLevel_TalonSrx.ParamEnum.eCustomParam1, out value);
            return HandleStatus(status);
        }
        public int IsPersStorageSaving(out bool isSaving)
        {
            int temp;
            int status = m_impl.RequestParam(LowLevel_TalonSrx.ParamEnum.ePersStorageSaving);
            HandleStatus(status);
            System.Threading.Thread.Sleep(kDelayForSolicitedSignalsMs); /* small yield for getting response */
            status = m_impl.GetParamResponseRaw(LowLevel_TalonSrx.ParamEnum.ePersStorageSaving, out temp);
            /* default to true if value is not sensble */
            isSaving = true;
            if (status == 0 && (temp == 0))
                isSaving = false;
            return HandleStatus(status);
        }
        public int SetCustomParam0(Int32 value, uint timeoutMs = 0)
        {
            return m_impl.SetParamRaw(LowLevel_TalonSrx.ParamEnum.eCustomParam0, value, timeoutMs);
        }
        public int SetCustomParam1(Int32 value, uint timeoutMs = 0)
        {
            return m_impl.SetParamRaw(LowLevel_TalonSrx.ParamEnum.eCustomParam1, value, timeoutMs);
        }

        /**
        * Common interface for inverting direction of a speed controller.
        * Only works in PercentVbus, speed, and Voltage modes.
        * @param isInverted The state of inversion, true is inverted.
        */
        public void SetInverted(bool isInverted) { m_isInverted = isInverted; }

        /**
         * Common interface for the inverting direction of a speed controller.
         *
         * @return isInverted The state of inversion, true is inverted.
         *
         */
        public bool GetInverted() { return m_isInverted; }

        /**
         * @return low level object for advanced control.
         */
        public CTRE.LowLevel_TalonSrx GetLowLevelObject() { return m_impl; }

        /// <summary>
        /// GetStatus on the gadgeteer UART connection to any intelligent Gadgeteer device (such as Pigeon).
        /// </summary>
        /// This implements the IGadgeteerUartClient interface.
        /// <param name="type"></param>
        /// <param name="connection"></param>
        /// <param name="bitrate"></param>
        /// <param name="resetCount"></param>
        /// <returns></returns>
        public int GetGadgeteerStatus(out GadgeteerProxyType type, out GadgeteerConnection connection, out int bitrate, out int resetCount)
        {
            return m_impl.GetGadgeteerStatus(out type, out connection, out bitrate, out resetCount);
        }
    }
}
