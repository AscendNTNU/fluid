//
// Created by simengangstad on 26.10.18.
//

#include "state.h"

#include <mavros_msgs/PositionTarget.h>
#include <utility>

#include "core.h"

fluid::State::State(std::string identifier,
                    std::string px4_mode,
                    bool steady, 
                    bool should_check_obstalce_avoidance_completion) : 

					identifier(identifier),
                    px4_mode(px4_mode),
                    steady(steady),
					pose_subscriber(node_handle.subscribe("mavros/local_position/pose", 
                                     Core::message_queue_size, 
                                     &State::poseCallback, 
                                     this)),
                    twist_subscriber(node_handle.subscribe("mavros/local_position/twist", 
                                      Core::message_queue_size,
                                      &State::twistCallback,
                                      this)),
                    setpoint_publisher(node_handle.advertise<mavros_msgs::PositionTarget>("fluid/setpoint", Core::message_queue_size)),
                    should_check_obstacle_avoidance_completion(should_check_obstacle_avoidance_completion) {}

geometry_msgs::PoseStamped fluid::State::getCurrentPose() const {
	return current_pose;
}

void fluid::State::poseCallback(const geometry_msgs::PoseStampedConstPtr pose) {
    current_pose.pose = pose->pose;
    current_pose.header = pose->header;
}

geometry_msgs::TwistStamped fluid::State::getCurrentTwist() const {
    return current_twist;
}

void fluid::State::twistCallback(const geometry_msgs::TwistStampedConstPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}

void fluid::State::publishSetpoint() {
    setpoint_publisher.publish(setpoint);
}

std::vector<std::vector<double>> fluid::State::getSplineForPath(const std::vector<geometry_msgs::Point>& path) const {
    // TODO: Temp, have to implement the call to the service

    std::vector<std::vector<double>> spline;

    double dx = path.front().x - getCurrentPose().pose.position.x;
    double dy = path.front().y - getCurrentPose().pose.position.y;
    double dz = path.front().z - getCurrentPose().pose.position.z;

    spline.push_back({dx, getCurrentPose().pose.position.x});
    spline.push_back({dy, getCurrentPose().pose.position.y});
    spline.push_back({dz, getCurrentPose().pose.position.z});

            // TODO: Add check to replan

            /*
            std::vector<float> x = {100.0f / factorial(13), 0, -100.0f / factorial(11), 0, 100.0f / factorial(9), 0, -100.0f / factorial(7), 0, 100.0f / factorial(5), 0.0, -100.0f / factorial(3), 0.0, 100, 0};
            std::vector<float> y = {10.0f / factorial(12), 0, -10.0f / factorial(10), 0, 10.0f / factorial(8), 0, -10.0f / factorial(6), 0, 10.0f / factorial(4), 0.0, -10.0f / factorial(2), 0, 10};
            std::vector<float> z = {0, 0, 0, 0, 0};
            std::vector<std::vector<float>> test_vec;
            test_vec.push_back(x);
            test_vec.push_back(y);
            test_vec.push_back(z);
            */


    return spline;
}

void fluid::State::perform(std::function<bool(void)> tick, bool should_halt_if_steady) {

    ros::Rate rate(Core::refresh_rate);

    initialize();

    ros::Time startTime = ros::Time::now();
    auto spline = getSplineForPath(path);

    while (ros::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && tick()) {

        ros::Duration current_time = ros::Time::now() - startTime;
        setpoint = Core::getControllerPtr()->getSetpoint(getPreferredController(), current_time.toSec(), spline);

        publishSetpoint();

        /*if (should_check_obstacle_avoidance_completion_ && obstacle_avoidance_completed_) {
            // Obstacle avoidance reported that we've come as far as we can in this state 
            ROS_INFO_STREAM(identifier << ": " << "OA completed");
            break;
        } */        


        fluid::Core::getStatusPublisherPtr()->status.path = path;
        fluid::Core::getStatusPublisherPtr()->publish();
        ros::spinOnce();
        rate.sleep();
   }
}