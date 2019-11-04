#ifndef FLUID_CONTROLLER_H
#define FLUID_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <mavros_msgs/PositionTarget.h>
#include <ascend_msgs/Spline.h>

#include "trajectory.h"
#include "PID.h"

namespace fluid {

    class Controller {

        private:

            PID pid;


        public:

            Controller(const PID& pid);

            mavros_msgs::PositionTarget getSetpoint(const Trajectory& trajectory,
                                                    const geometry_msgs::Pose& pose, 
                                                    const geometry_msgs::Twist& twist, 
                                                    const double& delta_time);
    };
}

#endif