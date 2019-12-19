#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "move_state.h"
#include "util.h"
#include "core.h"

bool fluid::MoveState::hasFinishedExecution() const {
    return been_to_all_points;
}

void fluid::MoveState::tick() {

    bool at_position_target  = Util::distanceBetween(getCurrentPose().pose.position, *current_destination_point_iterator) < fluid::Core::distance_completion_threshold;
    bool low_enough_velocity = std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold &&
                               std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold &&
                               std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;


    if (at_position_target && low_enough_velocity) {
        if (current_destination_point_iterator < path.end() - 1) {
            current_destination_point_iterator++;
            setpoint.position = *current_destination_point_iterator;
        }
        else {
            // We are at the final point
            been_to_all_points = true;
        }
    } 
}

void fluid::MoveState::initialize() {
    
    for (auto iterator = path.begin(); iterator != path.end(); iterator++) {
        if (iterator->z <= 0.1) {
            iterator->z = fluid::Core::default_height;
        }
    }

    setpoint.type_mask = TypeMask::Position;
    been_to_all_points = false;
    current_destination_point_iterator = path.begin();
}
