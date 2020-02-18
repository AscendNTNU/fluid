#include "follow_mast_state.h"

bool FollowMastState::hasFinishedExecution() const { return false; }

void FollowMastState::initialize() {

    setpoint.type_mask = TypeMask::Position;

    ascend_msgs::PositionYawTarget target;
    target.point = setpoint.position;
    target.yaw.data = 0;

    path.push_back(target);

}

void FollowMastState::modulePositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position) {

    module_info.pose = module_position->pose;
    
}

void FollowMastState::tick() {

    setpoint.position.x = module_info.pose.pose.position.y; //x,y is opposite from subscriber
    setpoint.position.y = module_info.pose.pose.position.x + 1;
    setpoint.position.z = module_info.pose.pose.position.z;

    double dx = module_info.pose.pose.position.y - getCurrentPose().pose.position.x;
    double dy = module_info.pose.pose.position.x - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);

    ascend_msgs::PositionYawTarget target;
    target.point = setpoint.position;
    target.yaw.data = setpoint.yaw;

    path.push_back(target);
    path.erase(path.begin());

}