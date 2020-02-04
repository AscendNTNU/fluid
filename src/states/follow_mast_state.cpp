#include "follow_mast_state.h"

bool FollowMastState::hasFinishedExecution() const { return false; }

void FollowMastState::initialize() {

    setpoint.type_mask = TypeMask::Position;

    path.push_back(module_info.pose.pose.position);

}

void FollowMastState::modulePositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position) {

    module_info.pose = module_position->pose;
    
}

void FollowMastState::tick() {

    setpoint.position.x = module_info.pose.pose.position.y;
    setpoint.position.y = module_info.pose.pose.position.x + 1;
    setpoint.position.z = module_info.pose.pose.position.z;

    path.push_back(setpoint.position);
    path.erase(path.begin());

    double dx = module_info.pose.pose.position.y - getCurrentPose().pose.position.x;
    double dy = module_info.pose.pose.position.x - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);

}