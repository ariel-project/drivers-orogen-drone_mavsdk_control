/* Generated from orogen/lib/orogen/templates/tasks/MavDroneControlTask.hpp */

#ifndef DRONE_MAVSDK_CONTROL_MAVDRONECONTROL_TASK_HPP
#define DRONE_MAVSDK_CONTROL_MAVDRONECONTROL_TASK_HPP

#include "drone_mavsdk_control/MavDroneControlTaskBase.hpp"

namespace drone_mavsdk_control
{

    /*! \class MavDroneControlTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine
to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These
interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the
associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/MavDroneControlTask.hpp and
tasks/MavDroneControlTask.cpp, and will be put in the mavsdk namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','mavsdk::MavDroneControlTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix
argument.
     */
    class MavDroneControlTask : public MavDroneControlTaskBase
    {
        friend class MavDroneControlTaskBase;

      protected:
      public:
        /** TaskContext constructor for MavDroneControlTask
         * \param name Name of the task. This name needs to be unique to make it
         * identifiable via nameservices. \param initial_state The initial TaskState of
         * the TaskContext. Default is Stopped state.
         */
        MavDroneControlTask(
            std::string const& name = "drone_mavsdk_control::MavDroneControlTask");

        /** Default deconstructor of MavDroneControlTask
         */
        ~MavDroneControlTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "MavDroneControlTaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
} // namespace drone_mavsdk_control

#endif
