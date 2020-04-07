/**
 * @file take_off_operation.cpp
 */

#include "take_off_operation.h"

#include "fluid.h"
#include "mavros_interface.h"
#include "util.h"

TakeOffOperation::TakeOffOperation(float height_setpoint)
    : Operation(OperationIdentifier::TAKE_OFF, false), height_setpoint(height_setpoint) {}

bool TakeOffOperation::hasFinishedExecution() const {
    const float distance_threshold = Fluid::getInstance().configuration.distance_completion_threshold;
    const float velocity_threshold = Fluid::getInstance().configuration.velocity_completion_threshold;
    return Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < 0.1 &&
           std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;
}

void TakeOffOperation::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.establishContactToPX4();
    Fluid::getInstance().getStatusPublisherPtr()->status.linked_with_px4 = 1;

    mavros_interface.requestArm(Fluid::getInstance().configuration.should_auto_arm);
    Fluid::getInstance().getStatusPublisherPtr()->status.armed = 1;

    mavros_interface.requestOffboard(Fluid::getInstance().configuration.should_auto_offboard);
    Fluid::getInstance().getStatusPublisherPtr()->status.px4_mode = PX4_MODE_OFFBOARD;

    setpoint.type_mask = TypeMask::IDLE;

    // Spin until we retrieve the first pose
    while (ros::ok() && getCurrentPose().header.seq == 0) {
        publishSetpoint();
        ros::spinOnce();
    }

    setpoint.position.x = getCurrentPose().pose.position.x;
    setpoint.position.y = getCurrentPose().pose.position.y;
    setpoint.position.z = height_setpoint;
    setpoint.yaw = getCurrentYaw();
    setpoint.type_mask = TypeMask::POSITION;
}
