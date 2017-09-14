using System;
using Microsoft.SPOT;

namespace CTRE
{
    public class Util
    {
        public static float Deadband(float value, float range)
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

        public static bool Contains(char[] array, char item)
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
