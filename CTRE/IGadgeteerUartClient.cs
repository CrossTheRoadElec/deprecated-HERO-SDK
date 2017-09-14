
using System;
using Microsoft.SPOT;

namespace CTRE
{
    public enum GadgeteerProxyType
    {
        General = 0,
        Pigeon = 1,
        PC_HERO = 2,
    }
    public enum GadgeteerConnection
    {
        NotConnected = 0,
        Connecting = 1,
        Connected = 2,
    }
    enum GadgeteerState
    {
        GadState_WaitChirp1 = 0,
        GadState_WaitBLInfo = 1,
        GadState_WaitBitrateResp = 2,
        GadState_WaitSwitchDelay = 3,
        GadState_WaitChirp2 = 4,
        GadState_Connected_Idle = 5,
        GadState_Connected_ReqChirp = 6,
        GadState_Connected_RespChirp = 7,
        GadState_Connected_ReqCanBus = 8,
        GadState_Connected_RespCanBus = 9,
        GadState_Connected_RespIsoThenChirp = 10,
        GadState_Connected_RespIsoThenCanBus = 11,
    }
    /// <summary>
    /// Interface for CAN Devices that allow for connecting intelligent UART Gadgeteer Devices.
    /// An example of this is usecase of plugging a ribbon cable between a Pigeon-IMU and Talon SRX (on CAN bus).
    /// Talon SRX would then implement this interface to provide status information on it's connection to the UART device.
    /// </summary>
    public interface IGadgeteerUartClient
    {
        int GetGadgeteerStatus(out GadgeteerProxyType type, out GadgeteerConnection connection, out int bitrate, out int resetCount);
    }
}
