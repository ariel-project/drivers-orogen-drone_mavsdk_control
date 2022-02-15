#include "MavDroneControlTask.hpp"

using namespace drone_mavsdk_control;
using namespace std;
using namespace mavsdk;
using namespace base;
using namespace power_base;
using namespace gps_base;
namespace dji = drone_dji_sdk;

MavDroneControlTask::MavDroneControlTask(std::string const& name)
    : MavDroneControlTaskBase(name)
{
}

MavDroneControlTask::~MavDroneControlTask() {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MavDroneControlTask.hpp for more detailed
// documentation about them.

typedef MavDroneControlTask::States TaskState;
static TaskState flightStatusToTaskState(Telemetry::LandedState status)
{
    switch (status)
    {
        case Telemetry::LandedState::Landing:
            return TaskState::LANDING;
        case Telemetry::LandedState::TakingOff:
            return TaskState::TAKING_OFF;
        case Telemetry::LandedState::InAir:
            return TaskState::IN_THE_AIR;
        case Telemetry::LandedState::OnGround:
            return TaskState::ON_THE_GROUND;
        case Telemetry::LandedState::Unknown:
            return TaskState::UNKNOWN;
    }
    // Never reached
    throw std::invalid_argument("invalid controller state");
}

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

static CommandResult convertToCommandResult(Mission::Result result)
{
    switch (result)
    {
        case Mission::Result::Unknown:
            return CommandResult::Unknown;
        case Mission::Result::Success:
            return CommandResult::Success;
        case Mission::Result::TransferCancelled:
            return CommandResult::MissionTransferCancelled;
        case Mission::Result::Busy:
            return CommandResult::Busy;
        default:
            throw std::invalid_argument(
                "Invalid mission result. It should probably have thrown an exception");
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
    mMission = unique_ptr<Mission>(new Mission(mSystem));
    reportCommand(DroneCommand::Config,
                  mAction->set_takeoff_altitude(_takeoff_altitude.get()));

    mUtmConverter.setParameters(_utm_parameters.get());
    mMaxDistanceFromSetpoint = _max_distance_from_setpoint.get();
    return true;
}
bool MavDroneControlTask::startHook()
{
    if (!MavDroneControlTaskBase::startHook())
        return false;

    mLastMission = dji::Mission();
    return true;
}
void MavDroneControlTask::updateHook()
{
    _unit_health.write(healthCheck(mTelemetry));
    _pose_samples.write(poseFeedback(mTelemetry));
    _battery.write(batteryFeedback(mTelemetry));

    TaskState status = flightStatusToTaskState(mTelemetry->landed_state());
    if (state() != status)
        state(status);

    dji::CommandAction cmd;
    if (_cmd_input.read(cmd) == RTT::NoData)
        return;

    // Hack to give command priority to external joystick controller
    if (mTelemetry->flight_mode() == Telemetry::FlightMode::Posctl)
        return;

    switch (cmd)
    {
        case dji::CommandAction::TAKEOFF_ACTIVATE:
        {
            dji::VehicleSetpoint setpoint;
            if (_cmd_pos.read(setpoint) != RTT::NewData)
                return;

            takeoffCommand(mTelemetry, mAction, setpoint);
            break;
        }
        case dji::CommandAction::LANDING_ACTIVATE:
        {
            dji::VehicleSetpoint setpoint;
            if (_cmd_pos.read(setpoint) != RTT::NewData)
                return;

            landingCommand(mTelemetry, mAction, setpoint);
            break;
        }
        case dji::CommandAction::GOTO_ACTIVATE:
        {
            dji::VehicleSetpoint setpoint;
            if (_cmd_pos.read(setpoint) != RTT::NewData)
                return;

            goToCommand(mTelemetry, mAction, setpoint);
            break;
        }
        case dji::CommandAction::MISSION_ACTIVATE:
        {
            dji::Mission mission_parameters;
            if (_cmd_mission.read(mission_parameters) != RTT::NewData)
                return;

            missionCommand(mMission, mission_parameters);
            break;
        }
    }
    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook() { MavDroneControlTaskBase::errorHook(); }
void MavDroneControlTask::stopHook() { MavDroneControlTaskBase::stopHook(); }
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

void MavDroneControlTask::takeoffCommand(unique_ptr<Telemetry> const& telemetry,
                                         unique_ptr<Action> const& action,
                                         dji::VehicleSetpoint const& setpoint)
{
    // Issue take off with a setpoint so the drone moves there
    // ASAP to avoid colision with the vessel.
    if (state() == TaskState::ON_THE_GROUND)
    {
        auto drone_arm_result = action->arm();
        reportCommand(DroneCommand::Arm, drone_arm_result);
        if (drone_arm_result == Action::Result::Success)
        {
            reportCommand(DroneCommand::Takeoff, action->takeoff());
        }
    }
    else if (state() == TaskState::IN_THE_AIR)
        goToCommand(telemetry, action, setpoint);
}

bool MavDroneControlTask::goToCommand(unique_ptr<Telemetry> const& telemetry,
                                      unique_ptr<Action> const& action,
                                      dji::VehicleSetpoint const& setpoint)
{
    Telemetry::Position mav_position = telemetry->position();
    Solution gps_position;
    gps_position.latitude = mav_position.latitude_deg;
    gps_position.longitude = mav_position.longitude_deg;
    gps_position.altitude = mav_position.absolute_altitude_m;
    Vector3d position = mUtmConverter.convertToNWU(gps_position).position;

    if ((position - setpoint.position).norm() < mMaxDistanceFromSetpoint)
        return true;

    Vector3d setpoint_absolute_altitude = setpoint.position;
    setpoint_absolute_altitude[2] += telemetry->home().absolute_altitude_m;

    samples::RigidBodyState setpoint_rbs;
    setpoint_rbs.position = setpoint_absolute_altitude;
    Solution gps_setpoint = mUtmConverter.convertNWUToGPS(setpoint_rbs);

    reportCommand(DroneCommand::Goto,
                  action->goto_location(gps_setpoint.latitude, gps_setpoint.longitude,
                                        gps_setpoint.altitude,
                                        -setpoint.heading.getDeg()));
    return false;
}

void MavDroneControlTask::landingCommand(unique_ptr<Telemetry> const& telemetry,
                                         unique_ptr<Action> const& action,
                                         dji::VehicleSetpoint const& setpoint)
{
    if (state() == TaskState::ON_THE_GROUND)
    {
        reportCommand(DroneCommand::Disarm, action->disarm());
    }
    else if (state() == TaskState::LANDING)
    {
        reportCommand(DroneCommand::Land, action->land());
    }
    else if (goToCommand(telemetry, action, setpoint))
    {
        reportCommand(DroneCommand::Land, action->land());
    }
}

void MavDroneControlTask::missionCommand(unique_ptr<Mission> const& mav_mission,
                                         dji::Mission const& mission_parameters)
{
    if (mLastMission == mission_parameters)
        return;
    mLastMission = mission_parameters;

    Mission::MissionPlan mission_plan = convert2MavMissionPlan(mission_parameters);
    auto upload_result = mav_mission->upload_mission(mission_plan);
    reportCommand(DroneCommand::MissionUpload, upload_result);
    reportCommand(DroneCommand::MissionStart, mav_mission->start_mission());
}

void MavDroneControlTask::reportCommand(DroneCommand const& command,
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

void MavDroneControlTask::reportCommand(DroneCommand const& command,
                                        Mission::Result const& result)
{
    switch (result)
    {
        case Mission::Result::NoSystem:
            throw DeviceError(
                "Mission command failed: could not find any system to issue command.");
        case Mission::Result::Error:
            throw CommandError("Mission command failed: unknown error reported.");
        case Mission::Result::Timeout:
            throw std::runtime_error("Mission command failed: timed out");
        case Mission::Result::InvalidArgument:
            throw CommandError("Mission command failed: invalid argument");
        case Mission::Result::TooManyMissionItems:
            throw CommandError("Mission command failed: too many items");
        case Mission::Result::UnsupportedMissionCmd:
        case Mission::Result::Unsupported:
            throw CommandError(
                "Mission command failed: this mission is not supported by the device");
        case Mission::Result::NoMissionAvailable:
            throw CommandError("Mission command failed: no missions available.");
        case Mission::Result::Unknown:
        case Mission::Result::Success:
        case Mission::Result::Busy:
        case Mission::Result::TransferCancelled:
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

Mission::MissionPlan
MavDroneControlTask::convert2MavMissionPlan(dji::Mission const& mission)
{
    Mission::MissionPlan plan;
    for (auto waypoint : mission.waypoints)
    {
        Mission::MissionItem mission_item;
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
    Quaterniond q_bodyned2ned(mav_orientation.w, mav_orientation.x, mav_orientation.y,
                              mav_orientation.z);
    Quaterniond q_ned2nwu = Quaterniond(AngleAxisd(M_PI, Vector3d::UnitX()));
    Quaterniond q_bodynwu2nwu = q_ned2nwu * q_bodyned2ned * q_ned2nwu.conjugate();
    pose.orientation = q_bodynwu2nwu;

    pose.time = base::Time::now();
    return pose;
}