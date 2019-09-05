#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/FluidAction.h>
#include <ascend_msgs/FluidGoal.h>

typedef actionlib::SimpleActionClient<ascend_msgs::FluidAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
			const ascend_msgs::FluidResultConstPtr &result) {
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO_STREAM("Final state of the drone: " << result->state.data << ". Pose of the drone: " << result->pose_stamped); 

	// Do something after we've reached the position sent in the goal
}

// Called once when the goal becomes active
void activeCb() {
	ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const ascend_msgs::FluidFeedbackConstPtr &feedback) {
	ROS_INFO_STREAM("Got Feedback! Current state: " << feedback->state << ". Current pose: " << feedback->pose_stamped);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_fibonacci_callback");

	// Create the action client
	std::string drone_namespace = "drone_1";
	Client action_client(drone_namespace + "/fluid_operation", true);

	ROS_INFO("Waiting for action server to start.");
	action_client.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	// Send Goal
	ascend_msgs::FluidGoal goal;

	// Can for example be take_off, land 
	goal.type.data = "move";
	goal.setpoint.x = 5.0;

	// As the drone is currently at the ground, sending this goal will make
	// the drone initialize, take off and move to the x position.
	// Since we didn't provide a z-value, an altitude, the state machine will
	// default to its defaultHeight param.
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	action_client.waitForResult();

	goal.setpoint.y = 5.0;
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

	ros::spin();

	return 0;
}