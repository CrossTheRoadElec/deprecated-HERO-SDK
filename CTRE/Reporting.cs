using System;
using Microsoft.SPOT;

namespace CTRE
{
    class Reporting
    {
        public static void SetError(int status, int unused)
        {
            SetError(status);
        }
        public static void SetError(int status)
        {
            if(status < 0)
                switch (status)
                {
                    case StatusCodes.PORT_MODULE_TYPE_MISMATCH:
                    {
                            Debug.Print("The selected Gadgeteer Port does not support the Socket Type required by this Module.");
                        break;
                    }
                    case StatusCodes.MODULE_NOT_INIT_SET_ERROR:
                    {
                            Debug.Print("The Module parameter cannot be set - Module is not initialized.");
                        break;
                    }
                    case StatusCodes.MODULE_NOT_INIT_GET_ERROR:
                    {
                            Debug.Print("The Module parameter could not be read - Module is not initialized.");
                        break;
                    }
                    default:
                    {
                        Debug.Print("CTRE.Reporting.SetError called with status " + status);
                        break;
                    }
            }
        }
        public static int getHALErrorMessage(int status)
        {
            return 0;
        }
    }
}
