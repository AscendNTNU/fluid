#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "move_state.h"
#include "util.h"
#include "core.h"

bool fluid::MoveState::hasFinishedExecution() const {
    // TODO: Have to check that we've been through the whole path
    bool atPositionTarget = Util::distanceBetween(getCurrentPose().pose.position, path.back()) < fluid::Core::distance_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;


    return atPositionTarget;
}

void fluid::MoveState::initialize() {
    
    for (auto iterator = path.begin(); iterator != path.end(); iterator++) {
        if (iterator->z <= 0.1) {
            iterator->z = fluid::Core::default_height;
        }
    }
}
