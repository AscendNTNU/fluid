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
    fluid::OperationClient init_operation_client(20);
    
    init_operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 0;
            take_off_pose.position.y = 0;
            take_off_pose.position.z = height;

            fluid::OperationClient take_off_operation_client(20);

            take_off_operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, take_off_pose, [&](bool completed) {
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
    fluid::OperationClient move_operation_client(60);
    
    float distance = 1;

    pose.position.x = distance;
    pose.position.y = 0;
    pose.position.z = height;
    
    move_operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
        if (completed) {
            pose.position.x = distance;
            pose.position.y = distance;
            pose.position.z = height;

            move_operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                if (completed) {

                    pose.position.x = 0;
                    pose.position.y = distance;
                    pose.position.z = height;

                    move_operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                        if (completed) {
                            pose.position.x = 0;
                            pose.position.y = 0;
                            pose.position.z = height;

                            move_operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                                if (completed) {
                                    fluid::OperationClient operation_land_client(60);

                                    geometry_msgs::Pose land_pose;
                                    land_pose.position.x = 0;
                                    land_pose.position.y = 0;
                                    land_pose.position.z = 0;

                                    operation_land_client.requestOperation(fluid::OperationIdentifier::Land, land_pose, [&](bool completed) {});

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