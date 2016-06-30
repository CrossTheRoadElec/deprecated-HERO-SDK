using Microsoft.SPOT.Hardware;

namespace CTRE
{
    namespace HERO
    {
        public class PortDefinition
        {
            public char[] types;
            public int id;
        }

        public interface IPortGpio3
        {
            Cpu.Pin Pin3 { get; }
            Cpu.Pin Pin4 { get; }
            Cpu.Pin Pin5 { get; }
        }

        public interface IPortGpio7
        {
            Cpu.Pin Pin3 { get; }
            Cpu.Pin Pin4 { get; }
            Cpu.Pin Pin5 { get; }
            Cpu.Pin Pin6 { get; }
            Cpu.Pin Pin7 { get; }
            Cpu.Pin Pin8 { get; }
            Cpu.Pin Pin9 { get; }
        }

        public interface IPortAnalog
        {
            Cpu.AnalogChannel Analog_Pin3 { get; }
            Cpu.AnalogChannel Analog_Pin4 { get; }
            Cpu.AnalogChannel Analog_Pin5 { get; }

            Cpu.Pin Pin3 { get; }
            Cpu.Pin Pin4 { get; }
            Cpu.Pin Pin6 { get; }
        }

        public interface IPortSDCard
        {
            //SD Card Definitions

            Cpu.Pin Pin3 { get; }
        }

        public interface IPortI2C
        {
            //I2C Definitions

            Cpu.Pin Pin3 { get; }
            Cpu.Pin Pin6 { get; }
        }

        public interface IPortUartHandshake
        {
            //Uart + Handshake Definitions

            Cpu.Pin Pin3 { get; }
        }

        public interface IPortAnalogOut
        {
            //Cpu.AnalogOutputChannel AnalogOut_Pin5 { get; }

            Cpu.Pin Pin3 { get; }
            Cpu.Pin Pin4 { get; }
        }

        public interface IPortPWM
        {
//            Cpu.PWMChannel PWM_Pin7 { get; }
//            Cpu.PWMChannel PWM_Pin8 { get; }
            Cpu.PWMChannel PWM_Pin9 { get; }
        }

        public interface IPortSPI { }

        public interface IPortUart
        {
            string UART { get; }

            Cpu.Pin Pin6 { get; }
        }

        

        public class Port1Definition : PortDefinition, IPortAnalog, IPortUart, IPortSPI, IPortGpio3
        {
            //These aren't officially part of the pin's types, but they're usable
            //public readonly Cpu.Pin Pin7 = (Cpu.Pin)0x4E;
            //public readonly Cpu.Pin Pin8 = (Cpu.Pin)0x4D;
            //public readonly Cpu.Pin Pin9 = (Cpu.Pin)0x4C;

            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x02; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x00; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x01; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x4A; } }

            public Cpu.AnalogChannel Analog_Pin3 { get { return Cpu.AnalogChannel.ANALOG_0; } }
            public Cpu.AnalogChannel Analog_Pin4 { get { return Cpu.AnalogChannel.ANALOG_1; } }
            public Cpu.AnalogChannel Analog_Pin5 { get { return Cpu.AnalogChannel.ANALOG_2; } }

            public string UART { get { return "COM4"; } }

            public Port1Definition()
            {
                types = new char[] { 'A', 'U', 'S', 'X' };
                id = 1;
            }
        }

        public class Port2Definition : PortDefinition
        {
            //public const Cpu.Pin port2_pin3
            public readonly Cpu.Pin Pin4 = (Cpu.Pin)0x09;
            public readonly Cpu.Pin Pin5 = (Cpu.Pin)0x0A;
            public readonly Cpu.Pin Pin6 = (Cpu.Pin)0x10;
            public readonly Cpu.Pin Pin7 = (Cpu.Pin)0x07;
            public readonly Cpu.Pin Pin8 = (Cpu.Pin)0x06;
            public readonly Cpu.Pin Pin9 = (Cpu.Pin)0x05;

            public Port2Definition()
            {
                types = new char[] { 'Z' };
                id = 2;
            }
        }

        public class Port3Definition : PortDefinition, IPortPWM, IPortGpio7
        {
            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x43; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x45; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x44; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x46; } }
            public Cpu.Pin Pin7 { get { return (Cpu.Pin)0x14; } }
            public Cpu.Pin Pin8 { get { return (Cpu.Pin)0x03; } }
            public Cpu.Pin Pin9 { get { return (Cpu.Pin)0x08; } }

            //public const Cpu.PWMChannel PWM_Pin7 = Cpu.PWMChannel.PWM_1;   //Not yet tested
            //public const Cpu.PWMChannel PWM_Pin8 = Cpu.PWMChannel.PWM_3;   //Not yet tested
            public Cpu.PWMChannel PWM_Pin9 { get { return Cpu.PWMChannel.PWM_2; } }

            public Port3Definition()
            {
                types = new char[] { 'P', 'Y' };
                id = 3;
            }
        }

        public class Port4Definition : PortDefinition, IPortI2C, IPortUartHandshake, IPortUart, IPortGpio3
        {
            //These aren't officially part of the pin's types, but they're usable
            public const Cpu.Pin Pin7 = (Cpu.Pin)0x33;
            //public const Cpu.Pin Pin8 = (Cpu.Pin)0x1B;
            //public const Cpu.Pin Pin9 = (Cpu.Pin)0x1A;

            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x37; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x35; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x36; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x34; } }

            public string UART { get { return "COM2"; } }

            public Port4Definition()
            {
                types = new char[] { 'I', 'K', 'U', 'X' };
                id = 4;
            }
        }

        public class Port5Definition : PortDefinition, IPortSDCard, IPortGpio7
        {
            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x30; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x28; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x29; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x32; } }
            public Cpu.Pin Pin7 { get { return (Cpu.Pin)0x2A; } }
            public Cpu.Pin Pin8 { get { return (Cpu.Pin)0x2B; } }
            public Cpu.Pin Pin9 { get { return (Cpu.Pin)0x2C; } }

            public Port5Definition()
            {
                types = new char[] { 'F', 'Y' };
                id = 5;
            }
        }

        public class Port6Definition : PortDefinition, IPortI2C, IPortUart, IPortGpio3
        {
            //These aren't officially part of the pin's types, but they're usable
            //public const Cpu.Pin Pin7  Not Connected
            //public const Cpu.Pin Pin8 = (Cpu.Pin)0x1B;
            //public const Cpu.Pin Pin9 = (Cpu.Pin)0x1A;

            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x41; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x26; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x27; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x40; } }

            public string UART { get { return "COM6"; } }

            public Port6Definition()
            {
                types = new char[] { 'I', 'U', 'X' };
                id = 6;
            }
        }

        public class Port7Definition : PortDefinition
        {
            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x22; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x3A; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x3F; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x3B; } }
            public Cpu.Pin Pin7 { get { return (Cpu.Pin)0x3D; } }
            public Cpu.Pin Pin8 { get { return (Cpu.Pin)0x3C; } }
            public Cpu.Pin Pin9 { get { return (Cpu.Pin)0x4B; } }

            public Port7Definition()
            {
                types = new char[] { 'Z' };
                id = 7;
            }
        }

        public class Port8Definition : PortDefinition, IPortAnalog, IPortAnalogOut, IPortSPI, IPortGpio3
        {
            //These aren't officially part of the pin's types, but they're usable
            //public const Cpu.Pin Pin7 = (Cpu.Pin)0x4E;
            //public const Cpu.Pin Pin8 = (Cpu.Pin)0x4D;
            //public const Cpu.Pin Pin9 = (Cpu.Pin)0x4C;

            public Cpu.Pin Pin3 { get { return (Cpu.Pin)0x20; } }
            public Cpu.Pin Pin4 { get { return (Cpu.Pin)0x21; } }
            public Cpu.Pin Pin5 { get { return (Cpu.Pin)0x04; } }
            public Cpu.Pin Pin6 { get { return (Cpu.Pin)0x3E; } }

            public Cpu.AnalogChannel Analog_Pin3 { get { return Cpu.AnalogChannel.ANALOG_3; } }
            public Cpu.AnalogChannel Analog_Pin4 { get { return Cpu.AnalogChannel.ANALOG_4; } }
            public Cpu.AnalogChannel Analog_Pin5 { get { return Cpu.AnalogChannel.ANALOG_5; } }

            public Port8Definition()
            {
                types = new char[] { 'A', 'O', 'S', 'X' };
                id = 8;
            }
        }

        public static class IO
        {
            public static Port1Definition Port1 = new Port1Definition();

            public static Port2Definition Port2 = new Port2Definition();

            public static Port3Definition Port3 = new Port3Definition();

            public static Port4Definition Port4 = new Port4Definition();

            public static Port5Definition Port5 = new Port5Definition();

            public static Port6Definition Port6 = new Port6Definition();

            public static Port7Definition Port7 = new Port7Definition();

            public static Port8Definition Port8 = new Port8Definition();
        }
    }
}