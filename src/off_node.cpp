#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    // Subscribe to state changes in order to check connection, arming and offboard flags.
    ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    ros::Publisher local_position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); // Publish our local position
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"); // Client for arming (?)
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"); // Client for set mode

    // Set point refreshing rate has to be faster than 2 Hz
    ros::Rate rate(20.0);

    // Wait for connection to establish between mavros and the autopilot
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Set initial position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // Before we enter offboard mode we have to start streaming data, 20 is an arbitrary number.
    for (int i = 20; ros::ok() && i > 0; --i) {
        local_position_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offboard_set_mode;
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arming_command;
    arming_command.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }

            last_request = ros::Time::now();
        }
        else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arming_command) && arming_command.response.success){
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();
            }
        }

        local_position_publisher.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}
