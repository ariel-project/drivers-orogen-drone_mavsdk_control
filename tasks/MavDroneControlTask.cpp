#include "MavDroneControlTask.hpp"

using namespace drone_mavsdk_control;
using namespace std;
using namespace mavsdk;
using namespace base;
using namespace power_base;
using namespace gps_base;
using namespace drone_control;

MavDroneControlTask::MavDroneControlTask(std::string const& name)
    : MavDroneControlTaskBase(name)
{
}

MavDroneControlTask::~MavDroneControlTask() {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MavDroneControlTask.hpp for more detailed
// documentation about them.

static CommandResult convertToCommandResult(Action::Result result)
{
    switch (result)
    {
        case Action::Result::Unknown:
            return CommandResult::Unknown;
        case Action::Result::Success:
            return CommandResult::Success;
        case Action::Result::Busy:
            return CommandResult::Busy;
        case Action::Result::CommandDenied:
            return CommandResult::CommandDenied;
        case Action::Result::CommandDeniedLandedStateUnknown:
            return CommandResult::CommandDeniedLandedStateUnknown;
        case Action::Result::CommandDeniedNotLanded:
            return CommandResult::CommandDeniedNotLanded;
        default:
            throw std::invalid_argument(
                "Invalid action result. It should probably have thrown an exception.");
    }
}

static CommandResult convertToCommandResult(mavsdk::Mission::Result result)
{
    switch (result)
    {
        case mavsdk::Mission::Result::Unknown:
            return CommandResult::Unknown;
        case mavsdk::Mission::Result::Success:
            return CommandResult::Success;
        case mavsdk::Mission::Result::TransferCancelled:
            return CommandResult::MissionTransferCancelled;
        case mavsdk::Mission::Result::Busy:
            return CommandResult::Busy;
        default:
            throw std::invalid_argument(
                "Invalid mission result. It should probably have thrown an exception");
    }
}

static CommandResult convertToCommandResult(Offboard::Result result)
{
    switch (result)
    {
        case Offboard::Result::Unknown:
            return CommandResult::Unknown;
        case Offboard::Result::Success:
            return CommandResult::Success;
        case Offboard::Result::CommandDenied:
            return CommandResult::CommandDenied;
        case Offboard::Result::Busy:
            return CommandResult::Busy;
        default:
            throw std::invalid_argument(
                "Invalid mission result. It should probably have thrown an exception");
    }
}

MavDroneControlTask::States
MavDroneControlTask::flightStatus(mavsdk::Telemetry::FlightMode flight_status)
{
    MavDroneControlTask::States current_state = state();
    switch (current_state)
    {
        case TELEMETRY:
        {
            if (_cmd_action.connected())
            {
                return CONTROLLING;
            }
            else
            {
                return TELEMETRY;
            }
        }
        case CONTROLLING:
        {
            bool can_take_control = canTakeControl(flight_status);
            if (!_cmd_action.connected())
            {
                return TELEMETRY;
            }
            else if (can_take_control)
            {
                return CONTROLLING;
            }
            else
            {
                return CONTROL_LOST;
            }
        }
        case CONTROL_LOST:
        {
            if (_cmd_action.connected())
            {
                return CONTROL_LOST;
            }
            else
            {
                return TELEMETRY;
            }
        }
        default:
            return current_state;
    }
}

bool MavDroneControlTask::configureHook()
{
    if (!MavDroneControlTaskBase::configureHook())
        return false;

    mMavHandler = unique_ptr<Mavsdk>(new Mavsdk());
    mMavHandler->set_timeout_s(_timeout.get().toSeconds());

    ConnectionResult connection_result = mMavHandler->add_any_connection(_address.get());
    if (connection_result != ConnectionResult::Success)
    {
        return false;
    }

    base::Time timeout_init = base::Time::now();
    while (mMavHandler->systems().size() == 0)
    {
        usleep(10000);
        if (base::Time::now() - timeout_init > base::Time::fromSeconds(1))
            return false;
    }
    mSystem = mMavHandler->systems().front();
    mTelemetry = unique_ptr<Telemetry>(new Telemetry(mSystem));
    mAction = unique_ptr<Action>(new Action(mSystem));
    mMission = unique_ptr<mavsdk::Mission>(new mavsdk::Mission(mSystem));
    mOffboard = unique_ptr<Offboard>(new Offboard(mSystem));
    reportCommand(
        DroneCommand::Config,
        mAction->set_takeoff_altitude(_takeoff_altitude.get()));

    mUtmConverter.setParameters(_utm_parameters.get());
    return true;
}

bool MavDroneControlTask::startHook()
{
    if (!MavDroneControlTaskBase::startHook())
        return false;

    mControllerStarted = Offboard::Result::Unknown;
    mLastMission = drone_control::Mission();
    mCurrentYaw = base::unknown<double>();
    state(TELEMETRY);

    return true;
}

void MavDroneControlTask::applyTransition(MavDroneControlTask::States const& next_state)
{
    if (next_state != CONTROLLING)
    {
        state(next_state);
        return;
    }

    if (!canTakeControl(mTelemetry->flight_mode()))
    {
        mAction->hold();
        sleep(1);
        state(CONTROLLING);
        return;
    }
    state(next_state);
}

void MavDroneControlTask::updateHook()
{
    _unit_health.write(healthCheck(mTelemetry));
    auto pose = poseFeedback(mTelemetry);
    mCurrentYaw = pose.getYaw();
    _pose_samples.write(pose);
    _battery.write(batteryFeedback(mTelemetry));
    _flight_status.write(flightStatusFeedback(mTelemetry));

    States status = flightStatus(mTelemetry->flight_mode());
    if (state() != status)
    {
        applyTransition(status);
    }

    if (state() == CONTROL_LOST)
        return;

    CommandAction cmd;
    if (_cmd_action.read(cmd) == RTT::NoData)
        return;

    switch (cmd)
    {
        case CommandAction::TAKEOFF_ACTIVATE:
        {
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;

            takeoffCommand(mTelemetry, mAction, mOffboard, setpoint);
            break;
        }
        case CommandAction::LANDING_ACTIVATE:
        {
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;

            landingCommand(mTelemetry, mAction, mOffboard, setpoint);
            break;
        }
        case CommandAction::POSITION_CONTROL_ACTIVATE:
        {
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;

            posCommand(mTelemetry, mOffboard, setpoint);
            break;
        }
        case CommandAction::VELOCITY_CONTROL_ACTIVATE:
        {
            VehicleSetpoint setpoint;
            if (_cmd_setpoint.read(setpoint) != RTT::NewData)
                return;

            velCommand(mOffboard, setpoint);
            break;
        }
        case CommandAction::MISSION_ACTIVATE:
        {
            drone_control::Mission mission_parameters;
            if (_cmd_mission.read(mission_parameters) != RTT::NewData)
                return;

            missionCommand(mMission, mission_parameters);
            break;
        }
    }
    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook() { MavDroneControlTaskBase::errorHook(); }
void MavDroneControlTask::stopHook()
{
    mOffboard->stop();
    MavDroneControlTaskBase::stopHook();
}
void MavDroneControlTask::cleanupHook() { MavDroneControlTaskBase::cleanupHook(); }

HealthStatus MavDroneControlTask::healthCheck(unique_ptr<Telemetry> const& telemetry)
{
    Telemetry::Health device_health = telemetry->health();

    if (!device_health.is_armable)
        mUnitHealth.status |= UnitHealth::NOT_ARMABLE;
    if (!device_health.is_home_position_ok)
        mUnitHealth.status |= UnitHealth::HOME_POSITION_NOT_SET;
    if (!device_health.is_global_position_ok)
        mUnitHealth.status |= UnitHealth::BAD_GLOBAL_POSITION_ESTIMATE;
    if (!device_health.is_local_position_ok)
        mUnitHealth.status |= UnitHealth::BAD_LOCAL_POSITION_ESTIMATE;
    if (!device_health.is_magnetometer_calibration_ok)
        mUnitHealth.status |= UnitHealth::UNCALIBRATED_MAGNETOMETER;
    if (!device_health.is_accelerometer_calibration_ok)
        mUnitHealth.status |= UnitHealth::UNCALIBRATED_ACCELEROMETER;
    if (!device_health.is_gyrometer_calibration_ok)
        mUnitHealth.status |= UnitHealth::UNCALIBRATED_GYROMETER;

    mUnitHealth.time = base::Time::now();
    return mUnitHealth;
}

void MavDroneControlTask::takeoffCommand(
    unique_ptr<Telemetry> const& telemetry,
    unique_ptr<Action> const& action,
    unique_ptr<Offboard> const& offboard,
    VehicleSetpoint const& setpoint)
{
    // Issue take off with a setpoint so the drone moves there
    // ASAP to avoid colision with the vessel.
    if (mTelemetry->landed_state() == Telemetry::LandedState::OnGround)
    {
        auto drone_arm_result = action->arm();
        reportCommand(DroneCommand::Arm, drone_arm_result);
        if (drone_arm_result == Action::Result::Success)
        {
            reportCommand(DroneCommand::Takeoff, action->takeoff());
        }
    }
    else if (mTelemetry->landed_state() == Telemetry::LandedState::InAir)
        posCommand(telemetry, offboard, setpoint);
}

bool MavDroneControlTask::posCommand(
    unique_ptr<Telemetry> const& telemetry,
    unique_ptr<Offboard> const& offboard,
    VehicleSetpoint const& setpoint)
{
    Vector3d setpoint_absolute_altitude = setpoint.position;
    setpoint_absolute_altitude[2] += telemetry->home().absolute_altitude_m;

    samples::RigidBodyState setpoint_rbs;
    setpoint_rbs.position = setpoint_absolute_altitude;
    Solution gps_setpoint = mUtmConverter.convertNWUToGPS(setpoint_rbs);

    Offboard::PositionGlobalYaw pos_cmd;
    pos_cmd.altitude_type = Offboard::PositionGlobalYaw::AltitudeType::Amsl;
    pos_cmd.lat_deg = gps_setpoint.latitude;
    pos_cmd.lon_deg = gps_setpoint.longitude;
    pos_cmd.alt_m = gps_setpoint.altitude;
    pos_cmd.yaw_deg = -setpoint.yaw.getDeg();

    reportCommand(DroneCommand::PosControl, offboard->set_position_global(pos_cmd));
    if (offboard->start() != Offboard::Result::Success)
        return false;

    return false;
}

bool MavDroneControlTask::velCommand(
    unique_ptr<Offboard> const& offboard,
    VehicleSetpoint const& setpoint)
{
    if (mControllerStarted == Offboard::Result::Unknown)
    {
        // Send it once before starting offboard, otherwise it will be rejected.
        Offboard::VelocityNedYaw stay{};
        offboard->set_velocity_ned(stay);
        mControllerStarted = offboard->start();
    }

    Offboard::VelocityNedYaw vel_cmd;
    vel_cmd.north_m_s = setpoint.velocity[0];
    vel_cmd.east_m_s = -setpoint.velocity[1];
    vel_cmd.down_m_s = -setpoint.velocity[2];
    vel_cmd.yaw_deg = - (mCurrentYaw + _K_yaw_speed2yaw.get() * setpoint.yaw_rate) * 180 / M_PI;
    reportCommand(DroneCommand::VelControl, offboard->set_velocity_ned(vel_cmd));

    if (mOffboard->start() != Offboard::Result::Success)
        return false;

    return true;
}

void MavDroneControlTask::landingCommand(
    unique_ptr<Telemetry> const& telemetry,
    unique_ptr<Action> const& action,
    unique_ptr<Offboard> const& offboard,
    VehicleSetpoint const& setpoint)
{
    if (mTelemetry->landed_state() == Telemetry::LandedState::OnGround)
    {
        reportCommand(DroneCommand::Disarm, action->disarm());
    }
    else if (mTelemetry->landed_state() == Telemetry::LandedState::Landing)
    {
        reportCommand(DroneCommand::Land, action->land());
    }
    else if (posCommand(telemetry, offboard, setpoint))
    {
        reportCommand(DroneCommand::Land, action->land());
    }
}

void MavDroneControlTask::missionCommand(
    unique_ptr<mavsdk::Mission> const& mav_mission,
    drone_control::Mission const& mission_parameters)
{
    if (mLastMission == mission_parameters)
        return;
    mLastMission = mission_parameters;

    mavsdk::Mission::MissionPlan mission_plan =
        convert2MavMissionPlan(mission_parameters);
    auto upload_result = mav_mission->upload_mission(mission_plan);
    reportCommand(DroneCommand::MissionUpload, upload_result);
    reportCommand(DroneCommand::MissionStart, mav_mission->start_mission());
}

void MavDroneControlTask::reportCommand(
    DroneCommand const& command,
    Action::Result const& result)
{
    switch (result)
    {
        case Action::Result::NoSystem:
            throw DeviceError(
                "Command failed: could not find any system to issue command.");
        case Action::Result::ConnectionError:
            throw DeviceError("Command failed: failed to connect with provided address");
        case Action::Result::ParameterError:
            throw CommandError("Command failed: error getting or setting parameter.");
        case Action::Result::Timeout:
            throw std::runtime_error("Command timed out.");
        case Action::Result::Unknown:
        case Action::Result::Success:
        case Action::Result::Busy:
        case Action::Result::CommandDenied:
        case Action::Result::CommandDeniedLandedStateUnknown:
        case Action::Result::CommandDeniedNotLanded:
        {
            CommandFeedback feedback;
            feedback.time = base::Time::now();
            feedback.command = command;
            feedback.result = convertToCommandResult(result);
            _command_feedback.write(feedback);
            break;
        }
        case Action::Result::NoVtolTransitionSupport:
        case Action::Result::VtolTransitionSupportUnknown:
            throw std::runtime_error("Unsupported result.");
    }
}

void MavDroneControlTask::reportCommand(
    DroneCommand const& command,
    mavsdk::Mission::Result const& result)
{
    switch (result)
    {
        case mavsdk::Mission::Result::NoSystem:
            throw DeviceError(
                "Mission command failed: could not find any system to issue command.");
        case mavsdk::Mission::Result::Error:
            throw CommandError("Mission command failed: unknown error reported.");
        case mavsdk::Mission::Result::Timeout:
            throw std::runtime_error("Mission command failed: timed out");
        case mavsdk::Mission::Result::InvalidArgument:
            throw CommandError("Mission command failed: invalid argument");
        case mavsdk::Mission::Result::TooManyMissionItems:
            throw CommandError("Mission command failed: too many items");
        case mavsdk::Mission::Result::UnsupportedMissionCmd:
        case mavsdk::Mission::Result::Unsupported:
            throw CommandError(
                "Mission command failed: this mission is not supported by the device");
        case mavsdk::Mission::Result::NoMissionAvailable:
            throw CommandError("Mission command failed: no missions available.");
        case mavsdk::Mission::Result::Unknown:
        case mavsdk::Mission::Result::Success:
        case mavsdk::Mission::Result::Busy:
        case mavsdk::Mission::Result::TransferCancelled:
        {
            CommandFeedback feedback;
            feedback.time = base::Time::now();
            feedback.command = command;
            feedback.result = convertToCommandResult(result);
            _command_feedback.write(feedback);
            break;
        }
    }
}

void MavDroneControlTask::reportCommand(
    DroneCommand const& command,
    Offboard::Result const& result)
{
    switch (result)
    {
        case Offboard::Result::NoSystem:
            throw DeviceError(
                "Offboard command failed: could not find any system to issue command.");
        case Offboard::Result::ConnectionError:
            throw DeviceError("Command failed: failed to connect with provided address");
        case Offboard::Result::Timeout:
            throw std::runtime_error("Offboard command failed: timed out");
        case Offboard::Result::NoSetpointSet:
            throw DeviceError("Command failed: Cannot start without setpoint set");
        case Offboard::Result::Success:
        case Offboard::Result::Unknown:
        case Offboard::Result::Busy:
        case Offboard::Result::CommandDenied:
        {
            CommandFeedback feedback;
            feedback.time = base::Time::now();
            feedback.command = command;
            feedback.result = convertToCommandResult(result);
            _command_feedback.write(feedback);
            break;
        }
    }
}

mavsdk::Mission::MissionPlan
MavDroneControlTask::convert2MavMissionPlan(drone_control::Mission const& mission)
{
    mavsdk::Mission::MissionPlan plan;
    for (auto waypoint : mission.waypoints)
    {
        mavsdk::Mission::MissionItem mission_item;
        samples::RigidBodyState position_rbs;
        position_rbs.position = waypoint.position;

        Solution gps_position = mUtmConverter.convertNWUToGPS(position_rbs);
        mission_item.latitude_deg = gps_position.latitude;
        mission_item.longitude_deg = gps_position.longitude;
        mission_item.relative_altitude_m = gps_position.altitude;
        mission_item.speed_m_s = mission.max_velocity;
        mission_item.gimbal_pitch_deg = waypoint.gimbal_pitch.getDeg();
        mission_item.is_fly_through = true;
        mission_item.yaw_deg = -waypoint.yaw.getDeg();
        plan.mission_items.push_back(mission_item);
    }
    return plan;
}

BatteryStatus MavDroneControlTask::batteryFeedback(unique_ptr<Telemetry> const& telemetry)
{
    // Get battery info. This method returns information about 1 battery.
    // I am not sure if this would be an issue with DJI drones (they send info
    // about batteries as a whole, no matter the number of batteries).
    Telemetry::Battery battery = telemetry->battery();
    BatteryStatus status;
    status.voltage = battery.voltage_v;
    status.charge = battery.remaining_percent;
    status.time = base::Time::now();
    return status;
}

samples::RigidBodyState
MavDroneControlTask::poseFeedback(unique_ptr<Telemetry> const& telemetry)
{
    Telemetry::Position mav_position = telemetry->position();
    Solution gps_position;
    gps_position.latitude = mav_position.latitude_deg;
    gps_position.longitude = mav_position.longitude_deg;
    gps_position.altitude = mav_position.absolute_altitude_m;

    samples::RigidBodyState pose = mUtmConverter.convertToNWU(gps_position);
    Telemetry::VelocityNed mav_vel = telemetry->velocity_ned();
    pose.velocity.x() = mav_vel.north_m_s;
    pose.velocity.y() = -mav_vel.east_m_s;
    pose.velocity.z() = -mav_vel.down_m_s;

    Telemetry::AngularVelocityBody mav_ang_vel =
        telemetry->attitude_angular_velocity_body();
    pose.angular_velocity.x() = mav_ang_vel.roll_rad_s;
    pose.angular_velocity.y() = -mav_ang_vel.pitch_rad_s;
    pose.angular_velocity.z() = -mav_ang_vel.yaw_rad_s;

    Telemetry::Quaternion mav_orientation = telemetry->attitude_quaternion();
    Quaterniond q_bodyned2ned(
        mav_orientation.w,
        mav_orientation.x,
        mav_orientation.y,
        mav_orientation.z);
    Quaterniond q_ned2nwu = Quaterniond(AngleAxisd(M_PI, Vector3d::UnitX()));
    Quaterniond q_bodynwu2nwu = q_ned2nwu * q_bodyned2ned * q_ned2nwu.conjugate();
    pose.orientation = q_bodynwu2nwu;

    pose.time = base::Time::now();
    return pose;
}

FlightStatus
MavDroneControlTask::flightStatusFeedback(unique_ptr<Telemetry> const& telemetry)
{
    switch (telemetry->landed_state())
    {
        case Telemetry::LandedState::OnGround:
            return FlightStatus::OnGround;
        case Telemetry::LandedState::TakingOff:
        case Telemetry::LandedState::Landing:
        case Telemetry::LandedState::InAir:
            return FlightStatus::Flying;
        default:
            return FlightStatus::Stopped;
    }
}

bool MavDroneControlTask::canTakeControl(mavsdk::Telemetry::FlightMode flight_status)
{
    return !(
        flight_status == Telemetry::FlightMode::Posctl ||
        flight_status == Telemetry::FlightMode::Altctl ||
        flight_status == Telemetry::FlightMode::Acro ||
        flight_status == Telemetry::FlightMode::Manual);
}