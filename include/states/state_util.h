//
// Created by simengangstad on 27.10.18.
//

#ifndef FLUID_FSM_STATE_UTIL_H
#define FLUID_FSM_STATE_UTIL_H

#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace fluid {
    class StateUtil {
    public:

        static std::string px4ModeForStateIdentifier(std::string state_identifier) {

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
                return "AUTO.LAND";
            }
        }

        static double distanceBetween(geometry_msgs::PoseStamped first, geometry_msgs::PoseStamped second) {
            double delta_x = second.pose.position.x - first.pose.position.x;
            double delta_y = second.pose.position.y - first.pose.position.y;
            double delta_z = second.pose.position.z - first.pose.position.z;

            return sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
        }
    };
}

#endif //FLUID_FSM_STATE_UTIL_H
