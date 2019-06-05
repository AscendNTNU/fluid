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

#include <fluid/client.h>
#include <fluid/state_identifier.h>
#include <fluid/state.h>

std::string name_space;
float xLength, yLength;
std::shared_ptr<fluid::Client> client_ptr;

bool initialPoseSet = false;
mavros_msgs::PositionTarget initialSetpoint, setpoint, lastSetpoint;

constexpr float height = 2.5;

void runOperation(/* bool completed ignored for this purpose */) {

    float x = initialSetpoint.position.x + (-xLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(xLength * 2))));
    float y = initialSetpoint.position.y + (-yLength + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(yLength * 2))));

    ROS_INFO_STREAM("Current pose: " << "(" << setpoint.position.x << ", " << setpoint.position.y << ", " << setpoint.position.z << ")");
    ROS_INFO_STREAM("Next position: " << "(" << x << ", " << y << ", " << setpoint.position.z << ")");

    float yaw = atan2(y - lastSetpoint.position.y, x - lastSetpoint.position.x);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
/*
    setpoint.orientation.x = quaternion.x();
    setpoint.orientation.y = quaternion.y();
    setpoint.orientation.z = quaternion.z();
    setpoint.orientation.w = quaternion.w();
*/
    setpoint.position.x = x;
    setpoint.position.y = y;
    setpoint.position.z = height;


    client_ptr->requestOperationToState(fluid::StateIdentifier::Move, setpoint,/* runOperation*/ [](bool completed) {});
    /* [=] (bool completed) {
        setpoint.position.x = x;
        setpoint.position.y = y;
        setpoint.position.z = height;

        operation_client_ptr->requestOperation(fluid::OperationIdentifier::Move, setpoint, runOperation);
    
        lastPose = setpoint;
    });*/
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {

    if (!initialPoseSet && fabs(pose->pose.position.x) > 0.5) {

        initialPoseSet = true;
        initialSetpoint.position = pose->pose.position;
        lastSetpoint = initialSetpoint;
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
    
    setpoint.position.x = initialSetpoint.position.x;
    setpoint.position.y = initialSetpoint.position.y;

    fluid::Client init_client(name_space, 60);

    init_client.requestTakeOff(height, [&](bool completed) {
        if (completed) {
            initialized = completed;
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

