#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "../include/core/operation/operation.h"
#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_defines.h"
#include "../include/core/state.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh("~");

    geometry_msgs::Pose pose;
    bool initialized = false;

    ROS_INFO("Starting init operation.");
    // Send an operation to initialize and arm the drone. Take off when this is done.
    fluid::OperationClient init_operation_client(20);
    init_operation_client.requestOperation(fluid::operation_identifiers::INIT, pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Init operation completed");

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 0;
            take_off_pose.position.y = 0;
            take_off_pose.position.z = 2;


            ROS_INFO("Starting take off operation.");
            fluid::OperationClient take_off_operation_client(20);

            take_off_operation_client.requestOperation(fluid::operation_identifiers::TAKE_OFF, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });


    // If we didn't manage to initialize, arm the drone and take off, abort.
    if (!initialized) {
        return 0;
    }

    ROS_INFO("Completed initialization and take off");

    // Just for demonstration, this will make the drone move in straight lines to form a square. When the current move
    // is finished, the next will execute as one can see in the callback.
    fluid::OperationClient move_operation_client(60);

    pose.position.x = 5;
    pose.position.y = 0;
    pose.position.z = 2;

    move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [&](bool completed) {
        if (completed) {
            ROS_INFO("Move operation completed");

            pose.position.x = 5;
            pose.position.y = 5;
            pose.position.z = 2;

            move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [&](bool completed) {
                if (completed) {
                    ROS_INFO("Move operation completed");

                    pose.position.x = 0;
                    pose.position.y = 5;
                    pose.position.z = 2;

                    move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [&](bool completed) {
                        if (completed) {
                            ROS_INFO("Move operation completed");

                            pose.position.x = 0;
                            pose.position.y = 0;
                            pose.position.z = 2;

                            move_operation_client.requestOperation(fluid::operation_identifiers::MOVE, pose, [&](bool completed) {
                                if (completed) {
                                    ROS_INFO("Move operation completed");

                                    fluid::OperationClient operation_land_client(60);

                                    geometry_msgs::Pose land_pose;
                                    land_pose.position.x = 0;
                                    land_pose.position.y = 0;
                                    land_pose.position.z = 0;

                                    operation_land_client.requestOperation(fluid::operation_identifiers::LAND, land_pose, [&](bool completed) {
                                        if (completed) {
                                            ROS_INFO("Land operation completed");
                                        }
                                    });

                                }
                            });
                        }
                    });
                }
            });
        }
    });


    return 0;
}

