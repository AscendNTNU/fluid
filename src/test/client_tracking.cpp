#include <thread>
#include <chrono>
#include <ostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <fluid/core/operation.h>
#include <fluid/core/client.h>
#include <fluid/operations/operation_identifier.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_tracking");
    ros::NodeHandle nh;

    geometry_msgs::Pose pose;
    bool initialized = false;
    float height = 1.0;

    fluid::Client client("drone_1", 60);
    
    client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            geometry_msgs::Pose take_off_pose;
            take_off_pose.position.x = 1;
            take_off_pose.position.y = 1;
            take_off_pose.position.z = height;

            client.requestOperation(fluid::OperationIdentifier::TakeOff, take_off_pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });

    ros::Rate rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        rate.sleep();
    }

    client.requestOperation(fluid::OperationIdentifier::PositionFollow, pose, [](bool completed) {});


    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
    
    while (ros::ok() && std::chrono::duration_cast<std::chrono::milliseconds>(current - startTime).count() < 50000) {
        current = std::chrono::system_clock::now();

        ros::spinOnce();
        rate.sleep();
    }

    pose.position.x = 1;
    pose.position.y = 1;
    pose.position.z = 0;

    client.requestOperation(fluid::OperationIdentifier::Land, pose, [](bool completed) {});

    ros::spin();

    return 0;
}

