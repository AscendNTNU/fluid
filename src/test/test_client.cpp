#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/FluidAction.h>
#include <ascend_msgs/FluidGoal.h>

typedef actionlib::SimpleActionClient<ascend_msgs::FluidAction> Client;

int main(int argc, char **argv) {

	ros::init(argc, argv, "test_client");

	Client client("drone_1/fluid_operation", true);
	ROS_INFO_STREAM("Waiting for server");
	client.waitForServer();
	ROS_INFO_STREAM("Got connection with server");

	ascend_msgs::FluidGoal goal;
	goal.type.data = "take_off";

	client.sendGoal(goal);
	client.waitForResult(ros::Duration(10.0));

	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Yay! The dishes are now clean");
	printf("Current State: %s\n", client.getState().toString().c_str());

	return 0;
}
