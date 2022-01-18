#include "MavDroneControlTask.hpp"

using namespace drone_mavsdk_control;

MavDroneControlTask::MavDroneControlTask(std::string const& name)
    : MavDroneControlTaskBase(name)
{
}

MavDroneControlTask::~MavDroneControlTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MavDroneControlTask.hpp for more detailed
// documentation about them.

bool MavDroneControlTask::configureHook()
{
    if (! MavDroneControlTaskBase::configureHook())
        return false;
    return true;
}
bool MavDroneControlTask::startHook()
{
    if (! MavDroneControlTaskBase::startHook())
        return false;
    return true;
}
void MavDroneControlTask::updateHook()
{
    MavDroneControlTaskBase::updateHook();
}
void MavDroneControlTask::errorHook()
{
    MavDroneControlTaskBase::errorHook();
}
void MavDroneControlTask::stopHook()
{
    MavDroneControlTaskBase::stopHook();
}
void MavDroneControlTask::cleanupHook()
{
    MavDroneControlTaskBase::cleanupHook();
}
