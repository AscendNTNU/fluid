#!/bin/bash


ROS_NAMESPACE=$1 roslaunch fluid_fsm server_gazebo.launch &
ROS_NAMESPACE=$1 rosrun Obst_Av obstacle_avoidance &
rosrun fluid_fsm client_random_movement $1 2.0 5.0
