#!/bin/bash


ROS_NAMESPACE=$1 roslaunch fluid_fsm server_gazebo.launch &
rosrun fluid_fsm client_random_movement $1 2.0 5.0