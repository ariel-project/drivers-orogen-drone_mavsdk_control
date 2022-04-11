#ifndef drone_mavsdk_control_TYPES_HPP
#define drone_mavsdk_control_TYPES_HPP

#include "base/Time.hpp"
/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace drone_mavsdk_control
{
    struct CommandError : public std::runtime_error
    {
        CommandError(std::string const& msg) : std::runtime_error(msg) {}
    };
    struct DeviceError : public std::runtime_error
    {
        DeviceError(std::string const& msg) : std::runtime_error(msg) {}
    };

    enum CommandResult
    {
        Unknown,
        Success,
        Busy,
        CommandDenied,
        CommandDeniedLandedStateUnknown,
        CommandDeniedNotLanded,
        MissionTransferCancelled
    };

    enum DroneCommand
    {
        Config,
        Arm,
        Disarm,
        Takeoff,
        Land,
        PosControl,
        VelControl,
        MissionUpload,
        MissionStart
    };

    struct CommandFeedback
    {
        base::Time time;
        DroneCommand command;
        CommandResult result;
    };

    struct HealthStatus
    {
        base::Time time;
        uint8_t status = 0;
    };
} // namespace drone_mavsdk_control

#endif
