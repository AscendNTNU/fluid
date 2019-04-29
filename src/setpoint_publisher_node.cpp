//
// Created by simengangstad on 29.04.19.
//

#include <ros/ros.h>
#include <string>
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::PositionTarget position_target;
bool position_is_set = false;


void subscriptionCallback(const mavros_msgs::PositionTarget::ConstPtr& pt) {
    position_target = *pt;
    position_is_set = true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm_setpoint_publisher");

    ROS_INFO("\nInitiailzing set point publisher.");

    int refersh_rate = static_cast<unsigned int>(atoi(argv[1]));
    std::string subscription_topic = std::string(argv[2]);
    std::string publishing_topic = std::string(argv[3]);

	ros::NodeHandle node_handle;
    node_handle.subscribe(subscription_topic, 100, subscriptionCallback);   


    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>(publishing_topic, 100);
    ros::Rate rate(refersh_rate);

    while (ros::ok()) {

        if (position_is_set) {
            publisher.publish(position_target);
        }
        
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
