using System;

namespace CTRE
{
    public class Gamepad
    {
        private ISingleGamepadValuesProvider _provider;
        private GamepadValues _values = new GamepadValues();

        private UInt32 _rumbleL = 0;
        private UInt32 _rumbleR = 0;
        private UInt32 _ledCode = 0;
        private UInt32 _controlFlags = 0;

        public Gamepad(ISingleGamepadValuesProvider provider)
        {
            _provider = provider;
        }
        /**
         * @param buttonIdx (One-indexed button).  '1' for button1 (first button).
         * @return true if specified button is true.
         */
        public bool GetButton(uint buttonIdx)
        {
            if (buttonIdx > 0)
                --buttonIdx;
            _provider.Get(ref _values);
            return (_values.btns >> (int)buttonIdx & 1) == 1;
        }
        /**
         * @param axisIdx (Zero-indexed axis).  '0' is typically the first X axis.
         * @return floating point value within [-1,1].
         */
        public float GetAxis(uint axisIdx)
        {
            _provider.Get(ref _values);
            if (axisIdx >= _values.axes.Length)
                return 0;
            return _values.axes[axisIdx];
        }
        /**
         * Retrieves a copy of the internal gamepadvalues structure used in decoding signals.
         * This can be used to retrieve signals not readily available through the Gamepad API (such as vendor specific signals or VID/PID).
         * To use this function, first create a gamepadValues instance and pass by reference.
         * <pre>{@code
         *      GamepadValues values = new GamepadValues(); // Create only once and use functiont to update it periodically.
         *      ...
         *      gamepad.GetAllValues(gamepadValues); // Get latest values
         * }</pre>
         * @param gamepadValues reference to update with latest values.
         * @return object reference to gamepadValues for function chaining.
         */
        public GamepadValues GetAllValues(ref GamepadValues gamepadValues)
        {
            /* get latest copy if there is new data */
            _provider.Get(ref _values);
            /* copy it to caller */
            gamepadValues.Copy(_values);
            return gamepadValues;
        }
        /**
         * Get the connection status of the Usb device.
         */
        public UsbDeviceConnection GetConnectionStatus()
        {
            int code = _provider.Get(ref _values);
            if (code >= 0)
                return UsbDeviceConnection.Connected;
            return UsbDeviceConnection.NotConnected;
        }

        /**
         * Set the Right Rumble strength.
		 * @param strength 0 for off, [1,255] 
		 * 			for on with increasing strength.
		 * @return int error code, 0 for success.
         */
        public int SetLeftRumble(byte strength)
        {
            _rumbleL = (uint)strength;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
        /**
         * Set the Right Rumble strength.
		 * @param strength 0 for off, [1,255] 
		 * 			for on with increasing strength.
		 * @return int error code, 0 for success.
         */
        public int SetRightRumble(byte strength)
        {
            _rumbleR = (uint)strength;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
        /**
         * Set the Left/Right Rumble strength at the same time.
		 * @param leftStrength 0 for off, [1,255] 
		 * 			for on with increasing strength.
		 * @param rightStrength 0 for off, [1,255] 
		 * 			for on with increasing strength.
		 * @return int error code, 0 for success.
         */
        public int SetRumble(byte leftStrength, byte rightStrength)
        {
            _rumbleL = (uint)leftStrength;
            _rumbleR = (uint)rightStrength;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
        /**
         * Set the Xbox LED code.
		 * @param Valid values are [6,9] for the four LEDs.
		 * @return int error code, 0 for success.
         */
        public int SetLEDCode(byte ledCode)
        {
            _ledCode = (uint)ledCode;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
        public int SetControlFlags(uint mask)
        {
            _controlFlags |= mask;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
        public int ClearControlFlags(uint mask)
        {
            _controlFlags &= ~mask;
            return _provider.Sync(ref _values, _rumbleL, _rumbleR, _ledCode, _controlFlags);
        }
    }
}
