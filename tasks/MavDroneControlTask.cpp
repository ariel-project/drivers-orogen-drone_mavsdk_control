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
        case Telemetry::LandedState::TakingOff:
        case Telemetry::LandedState::InAir:
            return TaskState::IN_THE_AIR;
        default:
            return TaskState::ON_THE_GROUND;
    }
    // Never reached
    throw std::invalid_argument("invalid controller state");
}

static CommandResult convertToCommandResult(Action::Result result)
{
    return (CommandResult)result;
}

static CommandResult convertToCommandResult(Mission::Result result)
{
    switch (result)
    {
        case Mission::Result::Error:
            return CommandResult::MissionError;
        case Mission::Result::TooManyMissionItems:
            return CommandResult::TooManyMissionItems;
        case Mission::Result::InvalidArgument:
            return CommandResult::MissionInvalidArgument;
        case Mission::Result::Unsupported:
            return CommandResult::UnsupportedMission;
        case Mission::Result::NoMissionAvailable:
            return CommandResult::NoMissionAvailable;
        case Mission::Result::UnsupportedMissionCmd:
            return CommandResult::UnsupportedMissionCmd;
        case Mission::Result::TransferCancelled:
            return CommandResult::MissionTransferCancelled;
        case Mission::Result::NoSystem:
            return CommandResult::NoSystem;
        case Mission::Result::Timeout:
            return CommandResult::Timeout;
        default:
            return (CommandResult)result;
    }
}

bool MavDroneControlTask::configureHook()
{
    if (!MavDroneControlTaskBase::configureHook())
        return false;
    Mavsdk mav_handler;
    mav_handler.set_timeout_s(_timeout.get().toSeconds());

    ConnectionResult connection_result = mav_handler.add_any_connection(_address.get());
    if (connection_result != ConnectionResult::Success)
    {
        LOG_ERROR("Unable to connect to device.")
        return false;
    }

    mTakeoffAltitude = _takeoff_altitude.get();
    mTakeoffFinishedThreshold = _takeoff_finished_threshold.get();

    mSystem = mav_handler.systems().front();
    auto action = Action(mSystem);
    action.set_takeoff_altitude(_takeoff_altitude.get());

    mUtmConverter.setParameters(_utm_parameters.get());
    return true;
}
bool MavDroneControlTask::startHook()
{
    if (!MavDroneControlTaskBase::startHook())
        return false;

    return true;
}
void MavDroneControlTask::updateHook()
{
    auto telemetry = Telemetry(mSystem);
    _unit_health.write(healthCheck(telemetry));
    _pose_samples.write(poseFeedback(telemetry));
    _battery.write(batteryFeedback(telemetry));

    TaskState status = flightStatusToTaskState(telemetry.landed_state());
    if (state() != status)
        state(status);

    dji::CommandAction cmd;
    if (_cmd_input.read(cmd) == RTT::NoData)
        return;

    switch (cmd)
    {
        case dji::CommandAction::TAKEOFF_ACTIVATE:
        {
            Action::Result takeoff_result = takeoffCommand(telemetry);
            _command_result.write(convertToCommandResult(takeoff_result));
            break;
        }
        case dji::CommandAction::LANDING_ACTIVATE:
        {
            Action::Result landing_result = landingCommand(telemetry);
            _command_result.write(convertToCommandResult(landing_result));
            break;
        }
        case dji::CommandAction::GOTO_ACTIVATE:
        {
            Action::Result goto_result = goToCommand();
            _command_result.write(convertToCommandResult(goto_result));
            break;
        }
        case dji::CommandAction::MISSION_ACTIVATE:
        {
            Mission::Result mission_result = missionCommand();
            _command_result.write(convertToCommandResult(mission_result));
            break;
        }
    }
    _command.write(mIssuedCmd);

    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook() { MavDroneControlTaskBase::errorHook(); }
void MavDroneControlTask::stopHook() { MavDroneControlTaskBase::stopHook(); }
void MavDroneControlTask::cleanupHook() { MavDroneControlTaskBase::cleanupHook(); }

HealthStatus MavDroneControlTask::healthCheck(Telemetry const& telemetry)
{
    Telemetry::Health device_health = telemetry.health();

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

Action::Result MavDroneControlTask::takeoffCommand(Telemetry const& telemetry)
{
    // Issue take off with a setpoint so the drone moves there
    // ASAP to avoid colision with the vessel.
    dji::VehicleSetpoint setpoint;
    if (_cmd_pos.read(setpoint) != RTT::NewData)
        return Action::Result::Unknown;

    Action action = Action(mSystem);
    if (telemetry.landed_state() == Telemetry::LandedState::OnGround)
    {
        auto drone_arm_result = action.arm();
        if (drone_arm_result == Action::Result::Success)
        {
            mIssuedCmd = DroneCommand::Takeoff;
            return action.takeoff();
        }
        else
        {
            mIssuedCmd = DroneCommand::Arm;
            return drone_arm_result;
        }
    }
    else
    {
        float current_altitude = telemetry.position().relative_altitude_m;
        if (abs(current_altitude - mTakeoffAltitude) < mTakeoffFinishedThreshold)
        {
            samples::RigidBodyState setpoint_rbs;
            setpoint_rbs.position = setpoint.position;

            Solution gps_setpoint = mUtmConverter.convertNWUToGPS(setpoint_rbs);
            mIssuedCmd = DroneCommand::Goto;
            return action.goto_location(gps_setpoint.latitude, gps_setpoint.longitude,
                                        gps_setpoint.altitude,
                                        -setpoint.heading.getDeg());
        }
        return Action::Result::Unknown;
    }
}

Action::Result MavDroneControlTask::goToCommand()
{
    dji::VehicleSetpoint setpoint;
    if (_cmd_pos.read(setpoint) != RTT::NewData)
        return Action::Result::Unknown;

    samples::RigidBodyState setpoint_rbs;
    setpoint_rbs.position = setpoint.position;

    Solution gps_setpoint = mUtmConverter.convertNWUToGPS(setpoint_rbs);
    Action action = Action(mSystem);
    mIssuedCmd = DroneCommand::Goto;
    return action.goto_location(gps_setpoint.latitude, gps_setpoint.longitude,
                                gps_setpoint.altitude, -setpoint.heading.getDeg());
}

Action::Result MavDroneControlTask::landingCommand(Telemetry const& telemetry)
{
    Action action = Action(mSystem);
    if (telemetry.landed_state() != Telemetry::LandedState::OnGround)
    {
        mIssuedCmd = DroneCommand::Land;
        return action.land();
    }
    else
    {
        mIssuedCmd = DroneCommand::Disarm;
        return action.disarm();
    }
}

Mission::Result MavDroneControlTask::missionCommand()
{
    dji::Mission mission;
    if (_cmd_mission.read(mission) != RTT::NewData)
        return Mission::Result::Unknown;

    Mission mav_mission = Mission(mSystem);
    Mission::MissionPlan mission_plan = convert2MavMissionPlan(mission);
    auto upload_result = mav_mission.upload_mission(mission_plan);
    if (upload_result == Mission::Result::Success)
    {
        mIssuedCmd = DroneCommand::MissionStart;
        return mav_mission.start_mission();
    }
    else
    {
        mIssuedCmd = DroneCommand::MissionUpload;
        return upload_result;
    }
}

Mission::MissionPlan
MavDroneControlTask::convert2MavMissionPlan(drone_dji_sdk::Mission const& mission)
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

BatteryStatus MavDroneControlTask::batteryFeedback(Telemetry const& telemetry)
{
    // Get battery info. This method returns information about 1 battery.
    // I am not sure if this would be an issue with DJI drones (they sent info
    // about the battery as a whole, no matter no number of batteries it has).
    Telemetry::Battery battery = telemetry.battery();
    BatteryStatus status;
    status.voltage = battery.voltage_v;
    status.charge = battery.remaining_percent;
    return status;
}

samples::RigidBodyState MavDroneControlTask::poseFeedback(Telemetry const& telemetry)
{
    Telemetry::Position mav_position = telemetry.position();
    Solution gps_position;
    gps_position.latitude = mav_position.latitude_deg;
    gps_position.longitude = mav_position.longitude_deg;
    gps_position.altitude = mav_position.absolute_altitude_m;

    samples::RigidBodyState pose = mUtmConverter.convertToNWU(gps_position);
    Telemetry::VelocityNed mav_vel = telemetry.velocity_ned();
    pose.velocity.x() = mav_vel.north_m_s;
    pose.velocity.y() = -mav_vel.east_m_s;
    pose.velocity.z() = -mav_vel.down_m_s;

    Telemetry::AngularVelocityBody mav_ang_vel =
        telemetry.attitude_angular_velocity_body();
    pose.angular_velocity.x() = mav_ang_vel.roll_rad_s;
    pose.angular_velocity.y() = -mav_ang_vel.pitch_rad_s;
    pose.angular_velocity.z() = -mav_ang_vel.yaw_rad_s;

    Telemetry::Quaternion mav_orientation = telemetry.attitude_quaternion();
    Quaterniond q_bodyned2ned(mav_orientation.w, mav_orientation.x, mav_orientation.y,
                              mav_orientation.z);
    Quaterniond q_ned2nwu(0, 1, 0, 0);
    Quaterniond q_bodynwu2nwu = q_ned2nwu * q_bodyned2ned * q_ned2nwu.conjugate();
    pose.orientation = q_bodynwu2nwu;

    pose.time = base::Time::now();
    return pose;
}