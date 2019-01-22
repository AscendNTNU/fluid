//
// Created by simengangstad on 22.11.18.
//

#ifndef FLUID_FSM_POSE_UTIL_H
#define FLUID_FSM_POSE_UTIL_H

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {
    class PoseUtil {
    public:
        static double distanceBetween(geometry_msgs::PoseStamped current, mavros_msgs::PositionTarget target) {
            double delta_x = target.position.x - current.pose.position.x;
            double delta_y = target.position.y - current.pose.position.y;
            double delta_z = target.position.z - current.pose.position.z;

            return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        }
    };
}

#endif //FLUID_FSM_POSE_UTIL_H
