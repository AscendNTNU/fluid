//
// Created by simengangstad on 07.03.19.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm_tracking_publisher");
    ros::NodeHandle node_handle;
    
    ros::Publisher publisher = node_handle.advertise<mavros_msgs::PositionTarget>("obstacle_avoidance/target_position", 100);

    mavros_msgs::PositionTarget setpoint;

    ros::Rate rate(5);

    double theta = 0.0;

    setpoint.position.z = 1.0;

    while (ros::ok()) {

        publisher.publish(setpoint);

        theta += 0.1;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
