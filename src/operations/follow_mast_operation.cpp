/**
 * @file follow_mast_operation.cpp
 */

#include "follow_mast_operation.h"

#include "operation_identifier.h"

FollowMastOperation::FollowMastOperation()
    : Operation(OperationIdentifier::FOLLOW_MAST, false),
      module_position_subscriber(
          node_handle.subscribe("/ai/ue4/module_pos", 10, &FollowMastOperation::modulePositionCallback, this)) {}

bool FollowMastOperation::hasFinishedExecution() const { return false; }

void FollowMastOperation::modulePositionCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position) {
    module_info.pose = module_position->pose;
}

void FollowMastOperation::tick() {
    setpoint.type_mask = TypeMask::POSITION;
    setpoint.position.x = module_info.pose.pose.position.y;
    // TODO: This has to be fixed, should be facing towards the module from any given position, not just from the x
    // direction
    setpoint.position.y = module_info.pose.pose.position.x + 1;
    setpoint.position.z = module_info.pose.pose.position.z;

    double dx = module_info.pose.pose.position.y - getCurrentPose().pose.position.x;
    double dy = module_info.pose.pose.position.x - getCurrentPose().pose.position.y;
    setpoint.yaw = std::atan2(dy, dx);
}