using System;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    class PWMSpeedController
    {
        private PWM controller = null;

        //Parameters for PWM Class.
        //Units are in Microseconds.
        private Cpu.PWMChannel channel;
        private uint period = 10000;
        private uint duration = 1500;
        private const PWM.ScaleFactor kScale = PWM.ScaleFactor.Microseconds;
        private const bool kInvertSignal = false;

        //Useful Motor Controller Class Attributes
        private int m_isInverted = 1; //1 is false and -1 is true.  Used as a multiplier.


        //Constructor for PWMController
        public PWMSpeedController(Cpu.PWMChannel channel)
        {
            CTRE.Native.Watchdog.RegisterSafeOutput(0);
            CTRE.Native.Watchdog.RegisterSafePWMOutput(0);

            if (0 == CTRE.Native.Watchdog.RegisterSafePWMOutput((uint)channel))
            {
                this.channel = channel;

                controller = new PWM(channel, period, duration, kScale, kInvertSignal);

                controller.Start();
            }
            else
            {
                /* could not safely register */
            }

        }

        //Overloaded constructor for different periods
        public PWMSpeedController(Cpu.PWMChannel channel, uint periodMs)
        {
            if (0 == CTRE.Native.Watchdog.RegisterSafePWMOutput((uint)channel))
            {
                this.channel = channel;

                controller = new PWM(channel, period, duration, kScale, kInvertSignal);

                SetPeriod(periodMs);

                controller.Start();
            }
            else
            {
                /* could not safely register */
            }
        }


        public bool Set(float percentVBus)
        {
            if (controller != null)
            {
                if (percentVBus > 1) percentVBus = 1;
                else if (percentVBus < -1) percentVBus = -1;

                //Centers duration around the neutral setpoint (1500) with a range +/- 500.
                //Also inverts the direction if necessary.
                duration = (uint)((m_isInverted * percentVBus * 500) + 1500);

                controller.Duration = duration;
                

                if (CTRE.Watchdog.IsEnabled())
                {
                    controller.Start(); //attempt to re-enable if back-end has disabled it 
                }
                return true;
            }
            return false;
        }

        public float Get()
        {
            return ((float)duration - 1500) / 500;
        }

        /*period input parameter is expected in milliseconds, as this is the standard   *
            * unit type expressed in data sheets.                                          */
        public bool SetPeriod(uint periodMs)
        {
            if (controller != null)
            {
                if (periodMs > 50) periodMs = 50;
                period = (periodMs * 1000);

                controller.Period = period;
                return true;
            }
            return false;
        }

        public uint GetPeriod() { return period; }

        public void Enable()
        {
            if (controller != null)
            {
                controller.Start();
            }
        }

        public void Disable()
        {
            if (controller != null)
            {
                controller.Stop();
            }
        }

        public void SetInverted(bool invert) { m_isInverted = invert ? -1 : 1; }

        public bool GetInverted() { return (m_isInverted == 1) ? false : true; }
    }

}
