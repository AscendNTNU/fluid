/**
 * @file take_off_state.cpp
 */

#include "take_off_state.h"
#include "core.h"
#include "util.h"
#include "mavros_interface.h"

TakeOffState::TakeOffState(float height_setpoint) : State(StateIdentifier::TAKE_OFF, false),
                                                    height_setpoint(height_setpoint) {}

bool TakeOffState::hasFinishedExecution() const {
    return Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < 0.1 &&
           std::abs(getCurrentTwist().twist.linear.x) < Core::velocity_completion_threshold &&
           std::abs(getCurrentTwist().twist.linear.y) < Core::velocity_completion_threshold &&
           std::abs(getCurrentTwist().twist.linear.z) < Core::velocity_completion_threshold;
}

void TakeOffState::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.establishContactToPX4();
    Core::getStatusPublisherPtr()->status.linked_with_px4 = 1;

    mavros_interface.requestArm(Core::auto_arm);
    Core::getStatusPublisherPtr()->status.armed = 1;

    mavros_interface.requestOffboard(Core::auto_set_offboard);
    Core::getStatusPublisherPtr()->status.px4_mode = "offboard";

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
