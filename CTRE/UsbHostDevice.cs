using System;

namespace CTRE
{
    public class UsbHostDevice : ISingleGamepadValuesProvider
    {
        static UsbHostDevice _instance;

        private int[] _gpInts = new int[20];
        private float[] _gpFlts = new float[6];
        int updateCnt = 0;

        /** keep private until we decide otherwise */
        private enum UsbHostDeviceMode
        { 
                Auto,
                Device,
                Host,
        }
        /** keep private until we decide otherwise */
        private static void SelectMode(UsbHostDeviceMode mode)
        {
            CTRE.Native.HDU.SetMode((uint)mode);
        }
        /** keep private until we decide otherwise */
        private static UsbHostDeviceMode GetSelectedMode()
        {
            return (UsbHostDeviceMode)CTRE.Native.HDU.GetMode();
        }
        /** 
         * Interface for anything that provides gamepad/joystick values (could be from a host pc or from USB attached gamepad). 
         * @return  Negative If values could not be retrieved due to connection issue.  toFill is cleared.
                    Zero if values are stale (no new data). toFill is left untouched.
         *          Positive if values are updated. toFill is filled in.
         */
        public int Get(ref GamepadValues toFill)
        {
            int ret = CTRE.Native.HDU.GetJoy(ref updateCnt, _gpInts, (uint)_gpInts.Length, _gpFlts, (uint)_gpFlts.Length);

            if(ret < 0)
            {
                /* negative error code means data is unreliable */
                toFill.Copy(GamepadValues.ZeroGamepadValues);
                /* on the next poll, always get latest */
                updateCnt = 0; 
            }
            else if(ret == 0)
            {
                /* no changes*/
            }
            else
            {
                /* new data, copy it over */
                toFill.axes[0] = _gpFlts[0];
                toFill.axes[1] = _gpFlts[1];
                toFill.axes[2] = _gpFlts[2];
                toFill.axes[3] = _gpFlts[3];
                toFill.axes[4] = _gpFlts[4];
                toFill.axes[5] = _gpFlts[5];
                toFill.btns = (uint)_gpInts[0];
                toFill.pov = _gpInts[1];
                toFill.vid = (uint)_gpInts[6];
                toFill.pid = (uint)_gpInts[7];
                toFill.vendorSpecF = _gpFlts;
                toFill.vendorSpecI = _gpInts;
            }
            return ret;
        }
        public static UsbHostDevice GetInstance()
        {
            if (_instance == null)
                _instance = new UsbHostDevice();
            return _instance;
        }
    }
}
