#include "follow_mast_state.h"

bool FollowMastState::hasFinishedExecution() const { return true; }

void FollowMastState::initialize() {

    setpoint.type_mask = TypeMask::Position;

}

void FollowMastState::modulePositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position) {

    module_info.pose = module_position->pose;
    
}

void FollowMastState::tick() {

    setpoint.position = module_info.pose.pose.position;

            double dx = module_info.pose.pose.position.x - getCurrentPose().pose.position.x;
            double dy = module_info.pose.pose.position.y - getCurrentPose().pose.position.y;
            setpoint.yaw = std::atan2(dy, dx);

}