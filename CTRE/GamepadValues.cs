using System;

namespace CTRE
{
    public class GamepadValues
    {
        public float [] axes = new float[6] { 0, 0, 0, 0, 0, 0 };
        public UInt32 btns = 0;          //!< Bitfield where '1' is pressed, '0' is not pressed.
        //public UInt32 btnChanges = 0;    //!< Bitfield where '1' means a button has transitioned not-pressed => pressed since last call.
        //public UInt32 btnsLast = 0;    //!< Bitfield where '1' means a button has transitioned not-pressed => pressed since last call.
        public Int32 pov = 0;           //!< -1 if POV is not pressed, degress when POV is used (0,45,90,135,180,225,270,315).
        public UInt32 vid = 0;
        public UInt32 pid = 0;
        public int[] vendorSpecI = null;
        public float[] vendorSpecF = null;
        public UInt32 flagBits = 0;
        
        public GamepadValues()
        {
            
        }
        public GamepadValues(GamepadValues rhs)
        {
            Copy(rhs);
        }
        public void Copy(GamepadValues rhs)
        {
            axes[0] = rhs.axes[0];
            axes[1] = rhs.axes[1];
            axes[2] = rhs.axes[2];
            axes[3] = rhs.axes[3];
            axes[4] = rhs.axes[4];
            axes[5] = rhs.axes[5];
            btns = rhs.btns;
            //btnChanges = rhs.btnChanges;
            //btnsLast = rhs.btnsLast;
            pov = rhs.pov;
            vid = rhs.vid;
            pid = rhs.pid;
            vendorSpecI = rhs.vendorSpecI;
            vendorSpecF = rhs.vendorSpecF;
            flagBits = rhs.flagBits;
            /* leave commands alone */
        }

        internal static GamepadValues ZeroGamepadValues = new GamepadValues();
    }
}
