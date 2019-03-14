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

    ros::Rate rate(5);

    double theta = 0.0;
    
    while (ros::ok()) {

        pose.position.x = 2*cos(theta);
        pose.position.y = 2*sin(theta);
        pose.position.z = 1.5;

    	publisher.publish(pose);

        theta += 0.1;

    	rate.sleep();
    	ros::spinOnce();
    }

    return 0;
}
