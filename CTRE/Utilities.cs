using System;
using Microsoft.SPOT;

namespace CTRE
{
    class Util
    {
        float Deadband(float value, float range)
        {
            if ( (value < -range) || (value > +range) )
            {
                return value;
                /* outside of deadband */
            }
            else
            {
                /* within range% so zero it */
                return 0;
            }
        }
    }
}
