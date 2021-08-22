/**
 * @file take_off_operation.cpp
 */

#include "take_off_operation.h"

#include "fluid.h"
#include "mavros_interface.h"
#include "util.h"
#include <mavros_msgs/CommandTOL.h>
#include <fluid/OperationCompletion.h>

TakeOffOperation::TakeOffOperation(float height_setpoint)
    : Operation(OperationIdentifier::TAKE_OFF, false, true), height_setpoint(height_setpoint) {}

bool TakeOffOperation::hasFinishedExecution() const {
    const float distance_threshold = Fluid::getInstance().configuration.distance_completion_threshold;
    const float velocity_threshold = Fluid::getInstance().configuration.velocity_completion_threshold;
    bool completed =  Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < distance_threshold &&
           std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;
    if (completed) {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": take_off OK!");
    }
    return completed;
}

bool cb(fluid::OperationCompletion::Request& request,
                                      fluid::OperationCompletion::Response& response) {
    return true;
}

void TakeOffOperation::initialize() {
    MavrosInterface mavros_interface;

    ros::Rate rate(5);
        
    mavros_interface.establishContactToArduPilot();
    Fluid::getInstance().getStatusPublisherPtr()->status.linked_with_ardupilot = 1;

    mavros_interface.requestArm(Fluid::getInstance().configuration.should_auto_arm);
    Fluid::getInstance().getStatusPublisherPtr()->status.armed = Fluid::getInstance().configuration.should_auto_arm;

    mavros_interface.requestOffboard(Fluid::getInstance().configuration.should_auto_offboard);
    Fluid::getInstance().getStatusPublisherPtr()->status.ardupilot_mode = ARDUPILOT_MODE_GUIDED;

    // Spin until we retrieve the first pose
    while(ros::ok() && getCurrentPose().header.seq == 0) {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "publish setPoint for takeoff\n");
        publishSetpoint();
        ros::spinOnce();
        rate.sleep();
    }

    int wpnav_speed_up = 2;
    mavros_interface.setParam("WPNAV_SPEED_UP", wpnav_speed_up);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat climb rate to: " << wpnav_speed_up/100.0 << " m/s.");

    //send take off command
    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = height_setpoint;
    setpoint.yaw = getCurrentYaw();
    // mavros_interface.requestTakeOff(setpoint);
}
