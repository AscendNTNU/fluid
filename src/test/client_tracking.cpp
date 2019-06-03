#include <thread>
#include <chrono>
#include <ostream>

#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <fluid/core/operation.h>
#include <fluid/core/client.h>
#include <fluid/core/state_identifier.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_tracking");
    ros::NodeHandle nh;

    mavros_msgs::PositionTarget setpoint;
    bool initialized = false;
    float height = 1.0;

    fluid::Client client("drone_1", 60);
    
    client.requestTakeOff(height, [&](bool completed) {
        if (completed) {
            initialized = completed;
        }
    });

    ros::Rate rate(20);

    while (ros::ok() && !initialized) {
        ros::spinOnce();
        rate.sleep();
    }

    client.requestOperationToState(fluid::StateIdentifier::PositionFollow, setpoint, [](bool completed) {});


    std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> current = std::chrono::system_clock::now();
    
    while (ros::ok() && std::chrono::duration_cast<std::chrono::milliseconds>(current - startTime).count() < 50000) {
        current = std::chrono::system_clock::now();

        ros::spinOnce();
        rate.sleep();
    }

    client.requestLand([](bool completed) {});

    ros::spin();

    return 0;
}

