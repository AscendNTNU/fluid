//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_STATE_UTIL_H
#define FLUID_FSM_STATE_UTIL_H

#include <string>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include "../core/state.h"

namespace fluid {
    class StateUtil {
    public:

        static std::string px4ModeForStateIdentifier(fluid::StateIdentifier state_identifier) {

            if (state_identifier == "idle") {
                return "OFFBOARD";
            }
            else if (state_identifier == "take_off") {
                return "OFFBOARD";
            }
            else if (state_identifier == "hold") {
                return "OFFBOARD";
            }
            else if (state_identifier == "move") {
                return "OFFBOARD";
            }
            else if (state_identifier == "land") {
                return "OFFBOARD";
            }
        }

        static double distanceBetween(geometry_msgs::PoseStamped current, mavros_msgs::PositionTarget target) {
            double delta_x = target.position.x - current.pose.position.x;
            double delta_y = target.position.y - current.pose.position.y;
            double delta_z = target.position.z - current.pose.position.z;

            return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        }
    };
}

#endif //FLUID_FSM_STATE_UTIL_H
