using System;

namespace CTRE
{
    public class PneumaticControlModule
    {
        private CTRE.LowLevel_Pcm m_impl;
        private bool[] _solen = new bool[16];
        /**
         * Constructor for the PneumaticControlModule device.
         * @param deviceNumber The CAN ID of the PCM
         */
        public PneumaticControlModule(int deviceNumber)
        {
            m_impl = new CTRE.LowLevel_Pcm((ushort)deviceNumber);
            m_impl.SetCloseLoopEnable(true);
        }
        /** report any comm/CAN errors if any occur */
        private void HandleErr(int status)
        {
            if (status != 0)
            {
                Reporting.SetError(status, Reporting.getHALErrorMessage(status));
            }
        }
        /**
         * Start the compressor running in closed loop control mode Use the method in
         * cases where you would like to manually stop and start the compressor for
         * applications such as conserving battery or making sure that the compressor
         * motor doesn't start during critical operations.
         */
        public void StartCompressor()
        {
            HandleErr(m_impl.SetCloseLoopEnable(true));
        }
        /**
         * Stop the compressor from running in closed loop control mode. Use the
         * method in cases where you would like to manually stop and start the
         * compressor for applications such as conserving battery or making sure that
         * the compressor motor doesn't start during critical operations.
         */
        public void StopCompressor()
        {
            HandleErr(m_impl.SetCloseLoopEnable(false));
        }

        /**
         * Get the current pressure switch value
         * @return true if the pressure is low by reading the pressure switch that is
         *         plugged into the PCM
         */
        public bool GetPressureSwitchValue()
        {
            bool retval;
            HandleErr(m_impl.GetPressureSwitchState(out retval));
            return retval;
        }
        /**
         * Get the current being used by the compressor
         * @return current consumed in amps for the compressor motor
         */
        public float GetCompressorCurrent()
        {
            float retval;
            HandleErr(m_impl.GetCompressorCurrent(out retval));
            return retval;
        }

        /**
         * @return true if PCM is in fault state : Compressor Drive is disabled due to
         * compressor current being too high.
         */
        public bool GetCompressorCurrentTooHighFault()
        {
            bool retval;
            HandleErr(m_impl.GetFault_CompCurrentTooHigh(out retval));
            return retval;
        }
        /**
         * @return true if PCM sticky fault is set : Compressor Drive is disabled due
         *         to compressor current being too high.
         */
        public bool GetCompressorCurrentTooHighStickyFault()
        {
            bool retval;
            HandleErr(m_impl.GetFault_CompCurrentTooHigh(out retval));
            return retval;
        }


        /**
         * @return true if PCM sticky fault is set : Compressor output appears to be
         *         shorted.
         */
        public bool GetCompressorShortedStickyFault()
        {
            bool retval;
            HandleErr(m_impl.GetStickyFault_CompShort(out retval));
            return retval;
        }

        /**
         * @return true if PCM is in fault state : Compressor output appears to be
         *         shorted.
         */
        public bool GetCompressorShortedFault()
        {
            bool retval;
            HandleErr(m_impl.GetFault_CompShort(out retval));
            return retval;
        }

        /**
         * @return true if PCM sticky fault is set : Compressor does not appear to be
         *         wired, i.e. compressor is not drawing enough current.
         */
        public bool GetCompressorNotConnectedStickyFault()
        {
            bool retval;
            HandleErr(m_impl.GetStickyFault_CompNoCurrent(out retval));
            return retval;
        }

        /**
         * @return true if PCM is in fault state : Compressor does not appear to be
         *         wired, i.e. compressor is not drawing enough current.
         */
        public bool GetCompressorNotConnectedFault()
        {
            bool retval;
            HandleErr(m_impl.GetFault_CompNoCurrent(out retval));
            return retval;
        }

        /**
         * Clear ALL sticky faults inside PCM that Compressor is wired to.
         *
         * If a sticky fault is set, then it will be persistently cleared. Compressor
         * drive maybe momentarily disable while flags are being cleared. Care should
         * be taken to not call this too frequently, otherwise normal compressor
         * functionality may be prevented.
         *
         * If no sticky faults are set then this call will have no effect.
         */
        public void ClearAllPCMStickyFaults()
        {
            HandleErr(m_impl.ClearStickyFaults());
        }

        public void SetSolenoidOutput(int idx, bool enable)
        {
            int status = m_impl.SetEnableSol(idx, enable);
            if (status == 0)
            {
                /* save the value so caller can check what the target output is */
                _solen[idx] = enable;
            }
            else
            {
                Reporting.SetError(status, Reporting.getHALErrorMessage(status));
            }
        }
        public bool GetSolenoidOutput(int idx)
        {
            if (idx < 0)
                return false;
            if (idx >= _solen.Length)
                return false;
            return _solen[idx];
        }
        public bool GetAppliedSolenoidOutput(int idx, out bool enable)
        {
            int status = m_impl.GetSol(idx, out enable);
            if (status != 0)
            {
                Reporting.SetError(status, Reporting.getHALErrorMessage(status));
            }
            return enable;
        }
        /**
           * Check if solenoid is blacklisted. If a solenoid is shorted, it is added to
           * the blacklist and disabled until power cycle, or until faults are cleared.
           *
           * @see #ClearAllPCMStickyFaults()
           *
           * @return If solenoid is disabled due to short.
           */
        public bool IsSolenoidBlackListed(int idx)
        {
            bool retval;
            HandleErr(m_impl.GetBlacklistSolen(idx, out retval));
            return retval;
        }
    }
}