#include "MavDroneControlTask.hpp"

using namespace drone_mavsdk_control;
using namespace std;
using namespace mavsdk;
using namespace drone_dji_sdk;

MavDroneControlTask::MavDroneControlTask(std::string const& name)
    : MavDroneControlTaskBase(name)
{
}

MavDroneControlTask::~MavDroneControlTask() {}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MavDroneControlTask.hpp for more detailed
// documentation about them.

bool MavDroneControlTask::configureHook()
{
    if (!MavDroneControlTaskBase::configureHook())
        return false;

    mTakeoffAltitude = _takeoff_altitude.get();
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
        return false;
    }

    mSystem = mMavSdk.systems().front();
    return true;
}
void MavDroneControlTask::updateHook()
{
    auto telemetry = Telemetry(mSystem);
    auto info = Info(mSystem);

    mStateFeedback.is_healthy = readyToTakeOff(telemetry);
    if (!mStateFeedback.is_healthy)
    {
        state(TaskState::NOT_READY);
        return;
    }

    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook() { MavDroneControlTaskBase::errorHook(); }
void MavDroneControlTask::stopHook() { MavDroneControlTaskBase::stopHook(); }
void MavDroneControlTask::cleanupHook() { MavDroneControlTaskBase::cleanupHook(); }

bool MavDroneControlTask::readyToTakeOff(Telemetry const& telemetry)
{
    Telemetry::Health device_health = telemetry.health();
    if (!device_health.is_gyrometer_calibration_ok)
    {
        LOG_ERROR("Gyro needs calibration!");
        return false;
    }
    if (!device_health.is_accelerometer_calibration_ok)
    {
        LOG_ERROR("Accelerometer needs calibration!");
        return false;
    }
    if (!device_health.is_magnetometer_calibration_ok)
    {
        LOG_ERROR("Compass needs calibration!");
        return false;
    }
    if (!device_health.is_local_position_ok)
    {
        LOG_ERROR("Local position estimation is not good enough to fly in 'position "
                  "control' mode");
        return false;
    }
    if (!device_health.is_global_position_ok)
    {
        LOG_ERROR("Global position estimation is not good enough to fly in 'position "
                  "control' mode");
        return false;
    }
    if (!device_health.is_armable)
    {
        LOG_ERROR("System is not armable!");
        return false;
    }
    return true;
}

MavDroneControlTask::TaskState MavDroneControlTask::runtimeStateTransition(TaskState const& current_state,
                                                      CommandAction const& command,
                                                      StateFeedback const& state_feedback)
{
    switch (current_state)
    {
    case TaskState::NOT_READY:
        if (state_feedback.is_healthy)
        {
            return TaskState::READY_TO_TAKE_OFF;
        }
        break;
    case TaskState::READY_TO_TAKE_OFF:
        if (command == CommandAction::TAKEOFF_ACTIVATE)
        {
            return TaskState::TAKING_OFF;
        }
        break;
    case TaskState::TAKING_OFF:
        if (mTakeoffAltitude - state_feedback.current_altitude <= 1e-3)
        {
            return TaskState::IN_THE_AIR;
        }
        break;
    case TaskState::IN_THE_AIR:
        switch (command)
        {
        case CommandAction::GO_TO_ACTIVATE:
        case CommandAction::MISSION_ACTIVATE:
            return TaskState::ON_MISSION;
        case CommandAction::PRE_LANDING_ACTIVATE:
            return TaskState::PREPARE_LANDING;
        default:
            break;
        }
        break;
    case TaskState::ON_MISSION:
        if (state_feedback.current_mission_finished)
        {
            return TaskState::IN_THE_AIR;
        }
        break;
    case TaskState::PREPARE_LANDING:
        if (state_feedback.current_mission_finished)
        {
            return TaskState::LANDING;
        }
        break;
    case TaskState::LANDING:
        if (state_feedback.landing_finished)
        {
            return TaskState::DISARMED;
        }
        break;
    case TaskState::DISARMED:
        if (state_feedback.disarm_finished)
        {
            return TaskState::READY_TO_TAKE_OFF;
        }
        break;
    default:
        break;
    }
    return current_state;
}