using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    public class PWMController
    {
        private PWM controller;

        //Parameters for PWM Class.
        //Units are in Microseconds.
        private Cpu.PWMChannel channel;
        private uint period = 20000;
        private uint duration = 1500;
        private const PWM.ScaleFactor kScale = PWM.ScaleFactor.Microseconds;
        private const bool kInvertSignal = false;

        //Useful Motor Controller Class Attributes
        private int m_isInverted = 1; //1 is false and -1 is true.  Used as a multiplier.

        
        //Constructor for PWMController
        public PWMController(Cpu.PWMChannel channel)
        {
            this.channel = channel;

            controller = new PWM(channel, period, duration, kScale, kInvertSignal);

            controller.Start();
        }


        public void Set(float percentVBus)
        {
            if (percentVBus > 1) percentVBus = 1;
            else if (percentVBus < -1) percentVBus = -1;

            //Centers duration around the neutral setpoint (1500) with a range +/- 500.
            //Also inverts the direction if necessary.
            duration = (uint)((m_isInverted * percentVBus * 500) + 1500);

            controller.Duration = duration;
        }

        public float Get()
        {
            return (controller.Duration - 1500)/500;
        }

        /*period input parameter is expected in milliseconds, as this is the standard   *
         * unit type expressed in data sheets.                                          */
        public void SetPeriod(uint period)  { controller.Period = this.period = (period * 1000); }

        public uint GetPeriod()  { return period; }

        public void Enable()  { controller.Start(); }

        public void Disable()  { controller.Stop(); }

        public void SetInverted(bool invert)  { m_isInverted = invert ? -1 : 1; }

        public bool GetInverted() { return (m_isInverted == 1) ? false : true; }


    }
}
