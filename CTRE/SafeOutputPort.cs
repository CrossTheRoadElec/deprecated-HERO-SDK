using Microsoft.SPOT.Hardware;
/**
 * @brief Digital Output that will safely transition to high-impedence if HERO is disabled.
 * @author Ozrien
 */
namespace CTRE
{
    public class SafeOutputPort
    {
        private Microsoft.SPOT.Hardware.TristatePort _out = null;

        public SafeOutputPort(Cpu.Pin portId, bool initialState) 
        {
            if (0 == Native.Watchdog.RegisterSafeOutput((uint)portId))
            {
                _out = new TristatePort(portId, initialState, false, Port.ResistorMode.Disabled);
            }
            else
            {
                /* could not safely register this digital output */
            }
        }
        /**
         * Attempt to update logic output (enable output if pin is high-Z).
         */
        public bool Write(bool state)
        {
            if (_out != null)
            {
                /* since the back-end may of disabled this output, attempt to re-enable */
                _out.Active = false;
                _out.Active = true;
                /* update the output latch */
                _out.Write(state);
                return true;
            }
            return false;
        }
    }
}