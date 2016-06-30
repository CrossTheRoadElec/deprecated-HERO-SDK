using System;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;

namespace CTRE
{
    namespace HERO
    {
        namespace Module
        {
            class DriverModule
            {
                public class state
                {
                    public readonly bool value;

                    public state(bool value)
                    {
                        this.value = value;
                    }
                }

                public static state driveLow = new state(true);
                public static state pullUp = new state(false);

                private const char kModulePortType = 'Y';

                private PortDefinition port;
                private OutputPort[] output = new OutputPort[7];

                public DriverModule(PortDefinition port)
                {
                   if (Contains(port.types, kModulePortType))
                    {
                        this.port = port;

                        InitializePort((IPortGpio7)this.port);
                    }
                }

                public void Set(int outputNum, state outputState)
                {
                    output[outputNum - 1].Write(outputState.value);
                }

                public state Get(int outputNum)
                {
                    if (output[outputNum].Read())
                        return driveLow;
                    else
                        return pullUp;
                }

                private void InitializePort(IPortGpio7 port)
                {
                    output[0] = new OutputPort(port.Pin3, pullUp.value);
                    output[1] = new OutputPort(port.Pin4, pullUp.value);
                    output[2] = new OutputPort(port.Pin5, pullUp.value);
                    output[3] = new OutputPort(port.Pin6, pullUp.value);
                    output[4] = new OutputPort(port.Pin7, pullUp.value);
                    output[5] = new OutputPort(port.Pin8, pullUp.value);
                }

                private bool Contains(char[] array, char item)
                {
                    bool found = false;

                    foreach (char element in array)
                    {
                        if (element == item)
                            found = true;
                    }

                    return found;
                }
            }
        }
    }
}
