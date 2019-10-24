#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ascend_msgs/FluidAction.h>
#include <ascend_msgs/FluidGoal.h>

typedef actionlib::SimpleActionClient<ascend_msgs::FluidAction> Client;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState &state,
			const ascend_msgs::FluidResultConstPtr &result) {
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	ROS_INFO_STREAM("Final state of the drone: " << result->state << ". Pose of the drone: " << result->pose_stamped); 

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
	Client action_client("fluid_operation", true);

	ROS_INFO("Waiting for action server to start.");
	action_client.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	// Send Goal
	ascend_msgs::FluidGoal goal;

	// The type of operation we want to execute. Can for example be:
	// - take_off
	// - land 
	// - move
	// - position_follow
	goal.state = "take_off";

	// As the drone is currently at the ground, sending this goal will make
	// the drone initialize and take off.
	// Since we didn't provide a setpoint with a z-value, an altitude, the state machine will
	// default to its defaultHeight param.
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
	action_client.waitForResult();

	// Send a new goal
	geometry_msgs::Point setpoint;
	setpoint.x = 5.0;
	goal.path = {setpoint};
	action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

	ros::spin();

	return 0;
}
