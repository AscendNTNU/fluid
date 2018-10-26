//
//  Created by Simen Gangstad on 15/10/2018.
//


#include "../../include/states/init_state.h"
#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class PosePublisher {
    virtual void publish(geometry_msgs::PoseStamped pose_stamped) = 0;
};

ros::NodeHandle nh;

class MavrosPublisher: public PosePublisher {
private:
    ros::Publisher local_position_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); // Publish our local position
public:

    void publish(geometry_msgs::PoseStamped pose_stamped) {
        local_position_publisher.publish(pose_stamped);
    }
};

class MavrosStateSubscriber {

public:
    mavros_msgs::State current_state;

    void state_callback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &MavrosStateSubscriber::state_callback, this);
};


class MavrosStateSetter {
public:

    MavrosStateSubscriber state_subscriber;

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"); // Client for set mode

    ros::Time last_request = ros::Time::now();

    std::string mode;
    mavros_msgs::SetMode set_mode;

    MavrosStateSetter(std::string mode) : mode(mode) {
        set_mode.request.custom_mode = mode;
    }


    void attemptToSetState() {
        if (state_subscriber.current_state.mode != mode && (ros::Time::now() - last_request > ros::Duration(5.0))) {

            if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
                ROS_INFO("%s enabled", mode.c_str());
            }

            last_request = ros::Time::now();
        }
    }
};

void fluid::InitState::perform() {
    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateBegan(*this);
    }

    MavrosPublisher publisher;
    MavrosStateSetter state_setter("OFFBOARD");

    ros::Rate rate(20); // Arbitrary number, must be higher than 2 Hz

    geometry_msgs::PoseStamped pose;

    // Run until we connect with mavros, init specific code
    {
        while (ros::ok() && !state_setter.state_subscriber.current_state.connected) {
            ros::spinOnce();
            rate.sleep();
        }

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;

        //send a few setpoints before starting. This is because the stream has to be set ut before we change modes within px4
        for (int i = 100; ros::ok() && i > 0; --i) {
            publisher.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }


    // Generic state code
    while(ros::ok()) {

        // Attempt to set mode

        state_setter.attemptToSetState();

        publisher.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }


    if (auto state_delegate = state_delegate_p.lock()) {
        state_delegate->stateFinished(*this);
    }
}
