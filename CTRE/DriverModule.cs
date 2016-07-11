using Microsoft.SPOT.Hardware;

namespace CTRE
{
    namespace HERO
    {
        namespace Module
        {
            public class DriverModule : ModuleBase
            {
                public class OutputState
                {
                    public const bool driveLow = true;
                    public const bool pullUp = false;
                }

                public readonly char kModulePortType = 'Y';

                private PortDefinition port;
                private OutputPort[] output = new OutputPort[7];
                private bool[] outputStates = new bool[7];
                private int status;

                public DriverModule(PortDefinition port)
                {
                   if (Contains(port.types, kModulePortType))
                    {
                        status = StatusCodes.OK;
                        this.port = port;
                        InitializePort((IPortGpio7)this.port);
                    }
                   else
                    {
                        status = StatusCodes.PORT_MODULE_TYPE_MISMATCH;
                        Reporting.SetError(status);
                    }
                }

                public void Set(int outputNum, bool outputState)
                {
                    if (status == StatusCodes.OK)
                    {
                        output[outputNum - 1].Write(outputState);
                        outputStates[outputNum - 1] = outputState;
                    }
                    else if (status == StatusCodes.PORT_MODULE_TYPE_MISMATCH)
                        Reporting.SetError(StatusCodes.MODULE_NOT_INIT_SET_ERROR);
                }

                public bool Get(int outputNum)
                {
                    if (status == StatusCodes.OK)
                        return outputStates[outputNum - 1];
                    else if (status == StatusCodes.PORT_MODULE_TYPE_MISMATCH)
                        Reporting.SetError(StatusCodes.MODULE_NOT_INIT_GET_ERROR);
                    
                    return false;
                }

                private void InitializePort(IPortGpio7 port)
                {
                    output[0] = new OutputPort(port.Pin3, OutputState.pullUp);
                    output[1] = new OutputPort(port.Pin4, OutputState.pullUp);
                    output[2] = new OutputPort(port.Pin5, OutputState.pullUp);
                    output[3] = new OutputPort(port.Pin6, OutputState.pullUp);
                    output[4] = new OutputPort(port.Pin7, OutputState.pullUp);
                    output[5] = new OutputPort(port.Pin8, OutputState.pullUp);

                    for (int i = 0; i < 6; i++)
                        outputStates[i] = false;

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
