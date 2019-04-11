#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;

    bool initialized = false;

    float height = 1.0;

    // Send an operation to initialize and arm the drone. Take off when this is done.
    fluid::OperationClient operation_client("drone_1", 20);
    
    operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 1;
            take_off_pose.position.y = 1;
            take_off_pose.position.z = height;

            operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });


    ros::Rate wait_rate(20);


    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    // Just for demonstration, this will make the drone move in straight lines to form a square. When the current move
    // is finished, the next will execute as one can see in the callback.
    // 
    float distance = 1;

    pose.position.x = 1 + distance;
    pose.position.y = 1;
    pose.position.z = height;
    
    operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
        if (completed) {
            pose.position.x = 1 + distance;
            pose.position.y = 1 + distance;
            pose.position.z = height;

            operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                if (completed) {

                    pose.position.x = 1;
                    pose.position.y = 1 + distance;
                    pose.position.z = height;

                    operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                        if (completed) {
                            pose.position.x = 1;
                            pose.position.y = 1;
                            pose.position.z = height;

                            operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                                if (completed) {
                                    geometry_msgs::Pose land_pose;
                                    land_pose.position.x = 1;
                                    land_pose.position.y = 1;
                                    land_pose.position.z = 0;

                                    operation_client.requestOperation(fluid::OperationIdentifier::Land, land_pose, [&](bool completed) {});

                                }
                            });
                        }
                    });
                }
            });
        }
    });

    ros::Rate rate(1);

    while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}