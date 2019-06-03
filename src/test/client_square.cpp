#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <fluid/core/client.h>
#include <fluid/core/operation_identifier.h>

#include <random>

fluid::Client client("drone_1", 15);

void foo() {

    geometry_msgs::Pose pose;

    pose.position.x = rand() % 10 - 5;
    pose.position.y = rand() % 10 - 5;
    pose.position.z = 1.0;

    client.requestOperation(fluid::OperationIdentifier::MoveOriented, pose, [&](bool completed) {
        foo();
    });
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_square");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;
    bool initialized = false;
    float height = 1.0;
    float startX = 0.0;
    float startY = 0.0;

    pose.position.x = pose.position.y = 5;

    client.requestOperation(fluid::OperationIdentifier::Land, pose, [&](bool completed) {
        if (completed) {
            initialized = completed;

            pose.position.x = pose.position.y = 0;

            client.requestOperation(fluid::OperationIdentifier::Land, pose, [](bool completed) {});
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
    float distance = 5;

    pose.position.x = startX + distance;
    pose.position.y = startY;
    pose.position.z = height;
    
    //foo();
    
    /*
    client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
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
    });*/

    ros::Rate rate(1);

    while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}
