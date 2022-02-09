#ifndef drone_mavsdk_control_TYPES_HPP
#define drone_mavsdk_control_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace drone_mavsdk_control {
    enum CommandResult{
        Unknown,
        Success,
        NoSystem,
        ConnectionError,
        Busy,
        CommandDenied,
        CommandDeniedLandedStateUnknown,
        CommandDeniedNotLanded,
        Timeout,
        ParameterError,
        MissionError,
        TooManyMissionItems,
        MissionInvalidArgument,
        UnsupportedMission,
        NoMissionAvailable,
        UnsupportedMissionCmd,
        MissionTransferCancelled,
        MissionNext
    };
}

#endif

