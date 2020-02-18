#include "init_state.h"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "core.h"
#include "mavros_state_link.h"
#include "util.h"

bool InitState::hasFinishedExecution() const { return initialized; }

void InitState::perform(std::function<bool(void)> tick, bool ignore_finished_execution) {
    ros::Rate rate(Core::refresh_rate);
    ros::NodeHandle node_handle_;

    // Establishing contact through mavros with Pixhawk.

    MavrosStateLink mavros_state_link;

    ROS_INFO("Attempting to establish contact with PX4...");

    // Run until we achieve a connection with mavros
    while (ros::ok() && !mavros_state_link.getCurrentState().connected) {
        Core::getStatusPublisherPtr()->publish();

        if (!tick()) {
            return;
        }

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    Core::getStatusPublisherPtr()->status.linked_with_px4 = 1;

    // send a few setpoints before starting. This is because the stream has to be set ut before we
    // change modes within px4
    mavros_msgs::PositionTarget setpoint;

    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 0;
    setpoint.type_mask = TypeMask::Idle;

    ascend_msgs::PositionYawTarget setpoint_yaw;
    setpoint_yaw.point = setpoint.position;
    setpoint_yaw.yaw.data = 0;

    for (int i = Core::refresh_rate * 2; ros::ok() && i > 0; --i) {
        publishSetpoint();
        Core::getStatusPublisherPtr()->status.path = {setpoint_yaw};
        Core::getStatusPublisherPtr()->publish();

        if (!tick()) {
            return;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Arming
    ROS_INFO_STREAM("Attemping to arm...");

    if (!Core::auto_arm) {
        ROS_INFO("Waiting for arm signal...");
    }

    ros::Time last_request = ros::Time::now();
    ros::ServiceClient arming_client = node_handle_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_command;
    arm_command.request.value = true;
    bool armed = false;
    double arm_request_interval = 0.5;

    while (ros::ok() && !hasFinishedExecution() && !armed) {
        if (!tick()) {
            return;
        }

        // Send request to arm every interval specified
        if (ros::Time::now() - last_request > ros::Duration(arm_request_interval)) {
            if (!mavros_state_link.getCurrentState().armed) {
                if (Core::auto_arm) {
                    if (arming_client.call(arm_command) && arm_command.response.success) {
                        Core::getStatusPublisherPtr()->status.armed = 1;
                        armed = true;
                    }
                }
            } else {
                Core::getStatusPublisherPtr()->status.armed = 1;
                armed = true;
            }

            last_request = ros::Time::now();
        }

        Core::getStatusPublisherPtr()->publish();
        publishSetpoint();

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    // Offboard
    ROS_INFO("Trying to set offboard...");

    if (!Core::auto_set_offboard) {
        ROS_INFO("Waiting for offboard signal...");
    }

    bool set_offboard = false;

    while (ros::ok() && !hasFinishedExecution() && !set_offboard) {
        if (!tick()) {
            return;
        }

        set_offboard = mavros_state_link.getCurrentState().mode == "OFFBOARD";

        if (Core::auto_set_offboard) {
            mavros_state_link.attemptToSetState("OFFBOARD", [&](bool completed) {
                set_offboard = completed;

                if (completed) {
                    Core::getStatusPublisherPtr()->status.px4_mode = "offboard";
                }
            });
        }

        Core::getStatusPublisherPtr()->publish();
        publishSetpoint();

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("OK!\n");

    initialized = true;
}
