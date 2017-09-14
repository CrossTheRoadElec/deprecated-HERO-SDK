using System;

namespace CTRE
{
    public class UsbHostDevice : ISingleGamepadValuesProvider
    {
        static UsbHostDevice _instance;

        private int[] _gpInts = new int[24];
        private float[] _gpFlts = new float[6];
        int updateCnt = 0;

        /**
		 * Mask bits for selecting with USB peripherals to allow
		 * on the host device port.
		 * BIT0 => HID devices (0 to enable).
		 * BIT1 => Selectable HID devices with switch set to 'D' (0 to enable).
		 * BIT2 => Selectable HID devices with switch set to 'X' (1 to enable).
		 * BIT3 => XInput/XBOX 360 Controllers. (0 to enable).
		 */
        private uint _maskBits = 0;

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

        public enum SelectableXInputFilter
        {
            DInputDevices,
            XInputDevices,
            BothDInputAndXInput,
        }
        /**
         * This will allow for Logitech gamepads with DInput selected.
         * Selecting XInput on the gamepad will cause joystick to 
         * disconnect (typically disabling robot).
         * This is the default setting for freshly powered HEROs, and also what 
         * is hardcoded in previous HERO releases.
         */
        //public static void EnableDInputDevices()
        //{
        //    _maskBits = 0;
        //}
        /**
         * This will allow for Logitech gamepads with XInput selected.
         * Selecting Dinput on the gamepad will cause joystick to 
         * disconnect (typically disabling robot).
         * This can be used to leverage the advanced features of the F710 series 
         * gamepad (analog triggers, vibration, more resolute axis data). 
         */
        //public static void EnableXInputDevices()
        //{
        //    _maskBits = 2 + 4;
        //}

        public void SetSelectableXInputFilter(SelectableXInputFilter filter)
        {
            switch (filter)
            {
                case SelectableXInputFilter.BothDInputAndXInput:
                    _maskBits = 4;
                    break;
                case SelectableXInputFilter.DInputDevices:
                    _maskBits = 0;
                    break;
                case SelectableXInputFilter.XInputDevices:
                    _maskBits = 2 + 4;
                    break;
            }
            /* ignore return */
            GamepadValues gv = null;
            Get(ref gv);
        }
        /** 
         * Interface for anything that provides gamepad/joystick values (could be from a host pc or from USB attached gamepad). 
         * @return  Negative If values could not be retrieved due to connection issue.  toFill is cleared.
                    Zero if values are stale (no new data). toFill is left untouched.
         *          Positive if values are updated. toFill is filled in.
         */

        private int SyncGet(ref GamepadValues toFill, uint numDwords)
        {
            /* always get latest data for now */
            updateCnt = 0;

            int ret = CTRE.Native.HDU.GetJoy(ref updateCnt, _gpInts, (uint)numDwords, _gpFlts, (uint)_gpFlts.Length);

            if(ret < 0)
            {
                /* negative error code means data is unreliable */
                if (toFill != null)
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
                if (toFill != null)
                {
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
                    toFill.flagBits = (uint)_gpInts[8];
                }
            }
            return ret;
        }


        public int Sync(ref GamepadValues toFill, uint rumbleL, uint rumbleR, uint ledCode, uint controlFlags)
        {
            /* save commands */
            _gpInts[20] = (int)rumbleL;
            _gpInts[21] = (int)rumbleR;
            _gpInts[22] = (int)ledCode;
            _gpInts[23] = (int)controlFlags;
            
            /* set the device mask bits, these are global */
            _gpInts[16] = (int)_maskBits;

            return SyncGet(ref toFill, (uint)_gpInts.Length);
        }
        public int Get(ref GamepadValues toFill)
        {
            /* set the device mask bits, these are global */
            _gpInts[16] = (int)_maskBits;

            const uint numDwords = 17; /* only send params to be read */

            return SyncGet(ref toFill, numDwords);
        }

        public static UsbHostDevice GetInstance()
        {
            if (_instance == null)
                _instance = new UsbHostDevice();
            return _instance;
        }
    }
}
