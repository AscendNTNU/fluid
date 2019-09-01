//
// Created by simengangstad on 29.04.19.
// 
// This node is a safety measure in case the main thread in the FSM is blocked and
// we therefore don't will publish setpoints to PX4 regularly. It grabs the last 
// published setpoint from the FSM and publishes that.
//

#include <ros/ros.h>
#include <string>
#include <ascend_msgs/ObstacleAvoidanceSetpoint.h>

#include "core.h"

ascend_msgs::ObstacleAvoidanceSetpoint obstacle_avoidance_setpoint;
bool position_is_set = false;

void subscriptionCallback(const ascend_msgs::ObstacleAvoidanceSetpoint::ConstPtr& pt) {
    obstacle_avoidance_setpoint = *pt;
    position_is_set = true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_setpoint_publisher");

    ROS_INFO("Initiailzing set point publisher.");

    int refresh_rate = fluid::Core::refresh_rate;
	ros::NodeHandle node_handle;

    node_handle.getParam("refresh_rate", refresh_rate);

    std::string subscription_topic = std::string(argv[1]);
    std::string publishing_topic = std::string(argv[2]);

    ros::Subscriber subscriber = node_handle.subscribe(subscription_topic, 1, subscriptionCallback);
    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>(publishing_topic, 1);
    ros::Rate rate(refresh_rate);

    while (ros::ok()) {

	obstacle_avoidance_setpoint.setpoint.header.stamp = ros::Time::now();

        if (position_is_set) {
            publisher.publish(obstacle_avoidance_setpoint.setpoint);
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
