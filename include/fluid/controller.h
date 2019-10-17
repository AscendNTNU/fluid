#ifndef FLUID_CONTROLLER_H
#define FLUID_CONTROLLER_H

#include <ros/ros.h>
#include <memory>
#include <vector>
#include <string>
#include <mavros_msgs/PositionTarget.h>
namespace fluid {
    class Controller {

        protected:
            const std::string topic_;
            const unsigned int degree_;
            ros::NodeHandle node_handle_;
            ros::Publisher setpoint_publisher_;

        public:

            Controller(const std::string& topic, const unsigned int& degree);
            void tick(std::shared_ptr<std::vector<std::vector<float>>> spline_ptr) const;
    };
}

#endif