using System;
/**
 * @brief Singleton Watchdog for ensuring actuators are disabled when appropriate.
 *      The simplest use case is for the user's application to call Watchdog.Feed() periodically
 *      to allow motor control.  This way if the managed app terminates, the motor drive will disable.
 *
 *      Similarly a developer can implement an Emergency-Stop or disable by connecting the desired
 *      switch or sensor to the logic so that Watchdog.Feed() is not called when sensor/switch is asserted.
 *
 *      For example an application can call Watchdog.Feed() only if a USB gamepad is inserted into the HERO 
 *      if that's what the intented behavior is.
 * @author Ozrien
 */
namespace CTRE
{
    public static class Watchdog
    {
        /**
         * Allow motor control for another 100ms.
         * Call this periodically to keep actuators enabled.
         */
        public static void Feed()
        {
            CTRE.Native.Watchdog.Feed(100);
        }
        public static void Feed(uint timeoutMs)
        {
            CTRE.Native.Watchdog.Feed(timeoutMs);
        }
        public static bool IsEnabled()
        {
            uint enabledBits = 0;
            return GetEnableBits(ref enabledBits);
        }
        public const uint kBit_ApiEnabled = 1;
        public const uint kBit_CcEnabled = 2;
        public const uint kBit_UsbGamepadEnabled = 4;

        public static bool GetEnableBits(ref uint enabledBits)
        {
            return CTRE.Native.Watchdog.GetEnableBits(ref enabledBits) == 0;
        }
    }
}