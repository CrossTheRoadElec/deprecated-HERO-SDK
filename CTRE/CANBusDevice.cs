using System;

namespace CTRE
{
    public abstract class CANBusDevice
    {
        protected uint _deviceNumber;

        public const UInt32 kFullMessageIDMask = 0x1fffffff;

        public CANBusDevice(uint deviceNumber)
        {
            _deviceNumber = deviceNumber;
        }
        public uint GetDeviceNumber()
        {
            return _deviceNumber;
        }

        public enum Codes
        {
            CAN_OK = 0,
            CAN_MSG_STALE = 1,
            CAN_TX_FULL = -1,
            CAN_INVALID_PARAM = -2,
            CAN_MSG_NOT_FOUND = -3,
            CAN_NO_MORE_TX_JOBS = -4,
            CAN_NO_SESSIONS_AVAIL = -5,
            CAN_OVERFLOW = -6,
            CAN_GENERAL_ERROR = -100,
            CTR_SigNotUpdated = -200,
        };
    }
}