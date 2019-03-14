//
// Created by simengangstad on 07.03.19.
//

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ascend_msgs/Persons.h>
#include <ascend_msgs/PersonDetection.h>
#include <ascend_msgs/BodyPartDetection.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");
    ros::NodeHandle node_handle;
    
    ros::Publisher publisher = node_handle.advertise<ascend_msgs::Persons>("perception/target", 100);

    ros::Rate rate(5);

    double theta = 0.0;
    
    while (ros::ok()) {
        ascend_msgs::Persons persons;
        ascend_msgs::PersonDetection personDetection;
        ascend_msgs::BodyPartDetection first_part, second_part;

        second_part.part_id = 1;
        second_part.x = 2*cos(theta);
        second_part.y = 2*sin(theta);

        personDetection.body_part.push_back(first_part);
        personDetection.body_part.push_back(second_part);
        persons.persons.push_back(personDetection);

        publisher.publish(persons);

        theta += 0.1;

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
