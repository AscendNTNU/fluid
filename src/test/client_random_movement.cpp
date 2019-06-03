#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <chrono>
#include <ostream>
#include <string>

#include <cstdlib> 
#include <ctime>     
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <chrono>
#include <ostream>

#include <fluid/core/client.h>
#include <fluid/core/operation_identifier.h>
#include <fluid/core/state.h>

std::string name_space;
float xLength, yLength;
std::shared_ptr<fluid::Client> client_ptr;

bool initialPoseSet = false;
geometry_msgs::Pose initialPose, pose, lastPose;

constexpr float height = 2.5;

void runOperation(/* bool completed ignored for this purpose */) {

    float x = initialPose.position.x + (-xLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(xLength * 2))));
    float y = initialPose.position.y + (-yLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(yLength * 2))));

    ROS_INFO_STREAM("Current pose: " << "(" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ")");
    ROS_INFO_STREAM("Next position: " << "(" << x << ", " << y << ", " << pose.position.z << ")");

    float yaw = atan2(y - lastPose.position.y, x - lastPose.position.x);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
/*
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
*/
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = height;


    client_ptr->requestOperation(fluid::OperationIdentifier::Move, pose,/* runOperation*/ [](bool completed) {});
    /* [=] (bool completed) {
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = height;

        operation_client_ptr->requestOperation(fluid::OperationIdentifier::Move, pose, runOperation);
    
        lastPose = pose;
    });*/
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {

    if (!initialPoseSet && fabs(pose->pose.position.x) > 0.5) {

        initialPoseSet = true;
        initialPose = pose->pose;
        lastPose = initialPose;
    }
}

int main(int argc, char** argv) {

    name_space = std::string(argv[1]);
    xLength = static_cast<float>(atof(argv[2]));
    yLength = static_cast<float>(atof(argv[3]));

    ros::init(argc, argv, name_space + "_random_movement_client");
    ros::NodeHandle node_handle;
    client_ptr = std::make_shared<fluid::Client>(name_space, 16);
    ros::Rate rate(20);

    // Retrieve initial pose 
    ros::Subscriber subscriber = node_handle.subscribe(name_space + "/mavros/local_position/pose", 100, poseCallback);

    while (ros::ok() && !initialPoseSet) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initialization and take off
    bool initialized = false;
    
    pose.position.x = initialPose.position.x;
    pose.position.y = initialPose.position.y;

    fluid::Client init_client(name_space, 60);

    init_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        if (completed) {

            pose.position.z = height;

            client_ptr->requestOperation(fluid::OperationIdentifier::TakeOff, pose, [&](bool completed) {
                initialized = completed;
            });
        }
    });


    std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
    
    while (ros::ok()) {
        current = std::chrono::system_clock::now();

        if (std::chrono::duration_cast<std::chrono::milliseconds>(current - lastTime).count() > 10000) {

            runOperation();
            lastTime = current;
        }

        ros::spinOnce();
        rate.sleep();
    }

    /*

    ros::spin();

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize the random movement
    srand(static_cast<unsigned int>(time(nullptr)));
    runOperation(true);
    ros::spin();
*/
    return 0;
}

