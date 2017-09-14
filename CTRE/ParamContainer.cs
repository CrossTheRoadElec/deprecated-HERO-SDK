/**
 * @brief TODO.
 */
using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    public class ParamContainer : CANBusDevice
    {
        private UInt32 PARAM_REQUEST;
        private UInt32 PARAM_RESPONSE;
        private UInt32 PARAM_SET;

        private UInt32 kParamArbIdValue;
        private UInt32 kParamArbIdMask;

        private UInt64 _cache;
        private UInt32 _len;
        private UInt32 _can_h = 0;
        private int _can_stat = 0;

        private System.Collections.Hashtable _sigs = new System.Collections.Hashtable();

        public ParamContainer(UInt32 deviceId, UInt32 paramReqId, UInt32 paramRespId, UInt32 paramSetId) : base(deviceId)
        {
            PARAM_REQUEST = paramReqId;
            PARAM_RESPONSE = paramRespId;
            PARAM_SET = paramSetId;

            kParamArbIdValue = PARAM_RESPONSE;
            kParamArbIdMask = 0xFFFFFFFF;
        }

        private void OpenSessionIfNeedBe()
        {
            _can_stat = 0;
            if (_can_h == 0)
            {
                /* bit30 - bit8 must match $000002XX.  Top bit is not masked to get remote frames */
                uint arbId = kParamArbIdValue | GetDeviceNumber();
                _can_stat = CTRE.Native.CAN.OpenStream(ref _can_h, kParamArbIdMask, arbId);
                if (_can_stat == 0)
                {
                    /* success */
                }
                else
                {
                    /* something went wrong, try again later */
                    _can_h = 0;
                }
            }
        }

        private void ProcessStreamMessages()
        {
            if (0 == _can_h) OpenSessionIfNeedBe();
            /* process receive messages */
            UInt32 i;
            UInt32 messagesRead = 0;
            UInt32 arbId = 0;
            UInt64 data = 0;
            UInt32 len = 0;
            UInt32 msgsRead = 0;
            /* read out latest bunch of messages */
            _can_stat = 0;
            if (_can_h != 0)
            {
                CTRE.Native.CAN.GetStreamSize(_can_h, ref messagesRead);
            }
            /* loop thru each message of interest */
            for (i = 0; i < messagesRead; ++i)
            {
                CTRE.Native.CAN.ReadStream(_can_h, ref arbId, ref data, ref len, ref msgsRead);
                if (arbId == (PARAM_RESPONSE | GetDeviceNumber()))
                {
                    byte paramEnum = (byte)(data & 0xFF);
                    data >>= 8;
                    /* save latest signal */
                    _sigs[(uint)paramEnum] = (uint)data;
                }
            }
        }

        /**
         * Send a one shot frame to set an arbitrary signal.
         * Most signals are in the control frame so avoid using this API unless you have
         * to.
         * Use this api for...
         * -A motor controller profile signal eProfileParam_XXXs.  These are backed up
         * in flash.  If you are gain-scheduling then call this periodically.
         * -Default brake and limit switch signals... eOnBoot_XXXs.  Avoid doing this,
         * use the override signals in the control frame.
         * Talon will automatically send a PARAM_RESPONSE after the set, so
         * GetParamResponse will catch the latest value after a couple ms.
         */
        public int SetParam(byte paramEnum, Int32 rawBits, uint timeoutMs = 0)
        {
            /* caller is using param API.  Open session if it hasn'T been done. */
            if (0 == _can_h) OpenSessionIfNeedBe();
            /* wait for response frame */
            if (timeoutMs != 0)
            {
                /* remove stale entry if caller wants to wait for response. */
                _sigs.Remove((uint)paramEnum);
            }
            /* frame set request and send it */
            UInt64 frame = ((UInt64)rawBits) & 0xFFFFFFFF;
            frame <<= 8;
            frame |= (byte)paramEnum;
            uint arbId = PARAM_SET | GetDeviceNumber();
            int status = CTRE.Native.CAN.Send(arbId, frame, 5, 0);
            /* wait for response frame */
            if (timeoutMs > 0)
            {
                int readBits;
                /* loop until timeout or receive if caller wants to check */
                while (timeoutMs > 0)
                {
                    /* wait a bit */
                    System.Threading.Thread.Sleep(1);
                    /* see if response was received */
                    if (0 == GetParamResponseRaw(paramEnum, out readBits))
                        break; /* leave inner loop */
                    /* decrement */
                    --timeoutMs;
                }
                /* if we get here then we timed out */
                if (timeoutMs == 0)
                    status = (int)Codes.CTR_SigNotUpdated;
            }
            return status;
        }

        public Int32 GetParam(byte paramEnum)
        {
            Int32 value;

            int status = RequestParam(paramEnum);

            System.Threading.Thread.Sleep(4); /* small yield for getting response */

            int statu2 = GetParamResponseRaw(paramEnum, out value);

            return value;
        }

        /**
         * Asks TALON to immedietely respond with signal value.  This API is only used
         * for signals that are not sent periodically.
         * This can be useful for reading params that rarely change like Limit Switch
         * settings and PIDF values.
          * @param param to request.
         */
        private int RequestParam(byte paramEnum)
        {
            /* process received param events. We don't expect many since this API is not
             * used often. */
            ProcessStreamMessages();
            int status = CTRE.Native.CAN.Send(PARAM_REQUEST | GetDeviceNumber(), (uint)paramEnum, 1, 0);
            return status;
        }

        /**
         * Checks cached CAN frames and updating solicited signals.
         */
        private int GetParamResponseRaw(byte paramEnum, out Int32 rawBits)
        {
            int retval = 0;
            /* process received param events. We don't expect many since this API is not
             * used often. */
            ProcessStreamMessages();
            /* grab the solicited signal value */
            if (_sigs.Contains((uint)paramEnum) == false)
            {
                retval = (int)Codes.CTR_SigNotUpdated;
                rawBits = 0; /* default value if signal was not received */
            }
            else
            {
                Object value = _sigs[(uint)paramEnum];
                uint temp = (uint)value;
                rawBits = (int)temp;
            }
            return retval;
        }


    }
}
