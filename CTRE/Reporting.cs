using System;
using Microsoft.SPOT;

namespace CTRE
{
    class Reporting
    {
        public static void SetError(int status, int unused)
        {
            if(status < 0)
            {
                Debug.Print("CTRE.Reporting.SetError called with status " + status);
            }
        }
        public static int getHALErrorMessage(int status)
        {
            return 0;
        }
    }
}
