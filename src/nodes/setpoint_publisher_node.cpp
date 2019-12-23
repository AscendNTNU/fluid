//
// This node is a safety measure in case the main thread in the FSM is blocked and
// we therefore don't will publish setpoints to PX4 regularly. It grabs the last
// published setpoint from the FSM and publishes that.
//

#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <string>

#include "core.h"

mavros_msgs::PositionTarget setpoint;
bool position_is_set = false;

void subscriptionCallback(const mavros_msgs::PositionTarget::ConstPtr& pt) {
    setpoint = *pt;
    position_is_set = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fluid_setpoint_publisher");

    ROS_INFO("Initiailzing set point publisher.");

    int refresh_rate = Core::refresh_rate;
    ros::NodeHandle node_handle;

    node_handle.getParam("refresh_rate", refresh_rate);

    std::string subscription_topic = std::string(argv[1]);
    std::string publishing_topic = std::string(argv[2]);

    ros::Subscriber subscriber = node_handle.subscribe(subscription_topic, 1, subscriptionCallback);
    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>(publishing_topic, 1);
    ros::Rate rate(refresh_rate);

    while (ros::ok()) {
        setpoint.header.stamp = ros::Time::now();

        if (position_is_set) {
            publisher.publish(setpoint);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
