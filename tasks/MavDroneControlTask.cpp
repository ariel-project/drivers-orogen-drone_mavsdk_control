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

bool MavDroneControlTask::configureHook()
{
    if (!MavDroneControlTaskBase::configureHook())
        return false;
    Mavsdk mav_handler;
    mav_handler.set_timeout_s(_timeout.get());

    ConnectionResult connection_result = mav_handler.add_any_connection(_address.get());
    if (connection_result != ConnectionResult::Success)
    {
        LOG_ERROR("Unable to connect to device.")
        return false;
    }

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
        issueTakeoffCommand();
    case dji::CommandAction::LANDING_ACTIVATE:
        landingCommand();
    case dji::CommandAction::GO_TO_ACTIVATE:
        goToCommand();
    case dji::CommandAction::MISSION_ACTIVATE:
        missionCommand();
    default:
        break;
    }

    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook() { MavDroneControlTaskBase::errorHook(); }
void MavDroneControlTask::stopHook() { MavDroneControlTaskBase::stopHook(); }
void MavDroneControlTask::cleanupHook() { MavDroneControlTaskBase::cleanupHook(); }

uint8_t MavDroneControlTask::healthCheck(Telemetry const& telemetry)
{
    Telemetry::Health device_health = telemetry.health();

    if (!device_health.is_armable)
        mUnitHealth |= UnitHealth::NOT_ARMABLE;
    if (!device_health.is_home_position_ok)
        mUnitHealth |= UnitHealth::HOME_POSITION_NOT_SET;
    if (!device_health.is_global_position_ok)
        mUnitHealth |= UnitHealth::BAD_GLOBAL_POSITION_ESTIMATE;
    if (!device_health.is_local_position_ok)
        mUnitHealth |= UnitHealth::BAD_LOCAL_POSITION_ESTIMATE;
    if (!device_health.is_magnetometer_calibration_ok)
        mUnitHealth |= UnitHealth::UNCALIBRATED_MAGNETOMETER;
    if (!device_health.is_accelerometer_calibration_ok)
        mUnitHealth |= UnitHealth::UNCALIBRATED_ACCELEROMETER;
    if (!device_health.is_gyrometer_calibration_ok)
        mUnitHealth |= UnitHealth::UNCALIBRATED_GYROMETER;

    return mUnitHealth;
}

void MavDroneControlTask::issueTakeoffCommand()
{
    // Take off should only be issued along a go to command,
    // to assure that the drone moves away from the boat ASAP.
    dji::VehicleSetpoint setpoint;
    if (_cmd_pos.read(setpoint) != RTT::NewData)
        return;

    auto noop = [](Action::Result result) {};
    Action action = Action(mSystem);
    action.arm();
    action.takeoff_async(noop);
}

void MavDroneControlTask::goToCommand()
{
    dji::VehicleSetpoint setpoint;
    if (_cmd_pos.read(setpoint) != RTT::NewData)
        return;

    samples::RigidBodyState setpoint_rbs;
    setpoint_rbs.position = setpoint.position;

    Solution gps_setpoint = mUtmConverter.convertNWUToGPS(setpoint_rbs);
    auto noop = [](Action::Result result) {};
    Action action = Action(mSystem);
    action.goto_location_async(gps_setpoint.latitude, gps_setpoint.longitude,
                               gps_setpoint.altitude, -setpoint.heading.getDeg(), noop);
}

void MavDroneControlTask::landingCommand()
{
    auto noop = [](Action::Result result) {};
    Action action = Action(mSystem);
    action.land_async(noop);
}

void MavDroneControlTask::missionCommand()
{
    dji::Mission mission;
    if (_cmd_mission.read(mission) != RTT::NewData)
        return;

    auto noop = [](Mission::Result result) {};
    Mission mav_mission = Mission(mSystem);
    Mission::MissionPlan mission_plan = djiMission2MavMissionPlan(mission);
    mav_mission.upload_mission_async(mission_plan, noop);
}

Mission::MissionPlan
MavDroneControlTask::djiMission2MavMissionPlan(drone_dji_sdk::Mission const& mission)
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
    pose.orientation.x() = mav_orientation.x;
    pose.orientation.y() = -mav_orientation.y;
    pose.orientation.z() = -mav_orientation.z;
    pose.orientation.w() = mav_orientation.w;

    pose.time = base::Time::now();
    return pose;
}