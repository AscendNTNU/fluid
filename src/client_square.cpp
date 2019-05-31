#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <tf2/LinearMath/Quaternion.h>

#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_square");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;
    bool initialized = false;
    float height = 1.0;
    float startX = 0.0;
    float startY = 0.0;

    fluid::OperationClient operation_client("drone_1", 10);
    
    operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        //if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = startX;
            take_off_pose.position.y = startY;
            take_off_pose.position.z = height;

            operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
//        }
    });


    ros::Rate wait_rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }

    // Just for demonstration, this will make the drone move in straight lines to form a square. When the current move
    // is finished, the next will execute as one can see in the callback.
    // 
    float distance = 5;

    pose.position.x = startX + distance;
    pose.position.y = startY;
    pose.position.z = height;
    
    operation_client.requestOperation(fluid::OperationIdentifier::MoveOriented, pose, [&](bool completed) {
        if (completed) {
                pose.position.x = startX + distance;
                pose.position.y = startY + distance;
                pose.position.z = height;

                operation_client.requestOperation(fluid::OperationIdentifier::MoveOriented, pose, [&](bool completed) {
                    if (completed) {

                        pose.position.x = startX;
                        pose.position.y = startY + distance;
                        pose.position.z = height;

                        operation_client.requestOperation(fluid::OperationIdentifier::MoveOriented, pose, [&](bool completed) {
                            if (completed) {
                                pose.position.x = startX;
                                pose.position.y = startY;
                                pose.position.z = height;

                                operation_client.requestOperation(fluid::OperationIdentifier::MoveOriented, pose, [&](bool completed) {
                                    if (completed) {
                                        geometry_msgs::Pose land_pose;
                                        land_pose.position.x = startX;
                                        land_pose.position.y = startY;
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
