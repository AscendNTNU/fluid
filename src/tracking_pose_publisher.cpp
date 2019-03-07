//
// Created by simengangstad on 07.03.19.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");
    ros::NodeHandle node_handle;

    ros::Publisher publisher = node_handle.advertise<geometry_msgs::Pose>("/perception/tracking", 100);

    geometry_msgs::Pose pose;

    pose.position.x = 5.0;
    pose.position.y = 5.0;
    pose.position.z = 1.5;

    ros::Rate rate(2);

    while (ros::ok()) {

    	publisher.publish(pose);

    	rate.sleep();
    	ros::spinOnce();
    }

    return 0;
}
