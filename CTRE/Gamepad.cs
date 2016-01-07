using System;

namespace CTRE
{
    public class Gamepad
    {
        private ISingleGamepadValuesProvider _provider;
        private GamepadValues _values = new GamepadValues();

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
    }
}
