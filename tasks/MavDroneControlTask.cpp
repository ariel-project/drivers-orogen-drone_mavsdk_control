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

    mUtmConverter.setParameters(_utm_parameters.get());
    return true;
}
bool MavDroneControlTask::startHook()
{
    if (!MavDroneControlTaskBase::startHook())
        return false;

    string connection_str =
        "serial://" + _serial_port.get() + ":" + to_string(_baudrate.get());
    ConnectionResult connection_result = mMavSdk.add_any_connection(connection_str);
    if (connection_result != ConnectionResult::Success)
    {
        LOG_ERROR("Unable to connect to device.")
        return false;
    }

    mSystem = mMavSdk.systems().front();
    auto telemetry = Telemetry(mSystem);
    healthCheck(telemetry);

    auto action = Action(mSystem);
    Action::Result action_result = action.arm();
    if (action_result != Action::Result::Success)
    {
        LOG_ERROR("Could not arm device.")
        return false;
    }
    action.set_takeoff_altitude(_takeoff_altitude.get());
    return true;
}
void MavDroneControlTask::updateHook()
{
    _pose_samples.write(poseFeedback());
    _battery.write(batteryFeedback());

    auto telemetry = Telemetry(mSystem);
    TaskState status = flightStatusToTaskState(telemetry.landed_state());
    if (state() != status)
        state(status);

    dji::CommandAction cmd;
    if (_cmd_input.read(cmd) == RTT::NoData)
    {
        return;
    }

    switch (cmd)
    {
    case dji::CommandAction::TAKEOFF_ACTIVATE:
        issueTakeoffCommand();
    case dji::CommandAction::PRE_LANDING_ACTIVATE:
        // Issue a go to command to hover near Tupan
        goToCommand();
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

void MavDroneControlTask::healthCheck(Telemetry const& telemetry)
{
    Telemetry::Health device_health = telemetry.health();
    if (!device_health.is_gyrometer_calibration_ok)
    {
        LOG_ERROR("Gyro needs calibration!");
    }
    if (!device_health.is_accelerometer_calibration_ok)
    {
        LOG_ERROR("Accelerometer needs calibration!");
    }
    if (!device_health.is_magnetometer_calibration_ok)
    {
        LOG_ERROR("Compass needs calibration!");
    }
    if (!device_health.is_local_position_ok)
    {
        LOG_ERROR("Local position estimation is not good enough to fly in 'position "
                  "control' mode");
    }
    if (!device_health.is_global_position_ok)
    {
        LOG_ERROR("Global position estimation is not good enough to fly in 'position "
                  "control' mode");
    }
    if (!device_health.is_armable)
    {
        LOG_ERROR("System is not armable!");
    }
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

BatteryStatus MavDroneControlTask::batteryFeedback()
{
    auto telemetry = Telemetry(mSystem);
    // Get battery info. This method returns information about 1 battery.
    // I am not sure if this would be an issue with DJI drones (they sent info
    // about the battery as a whole, no matter no number of batteries it has).
    Telemetry::Battery battery = telemetry.battery();
    BatteryStatus status;
    status.voltage = battery.voltage_v;
    status.charge = battery.remaining_percent;
    return status;
}

samples::RigidBodyState MavDroneControlTask::poseFeedback()
{
    auto telemetry = Telemetry(mSystem);
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