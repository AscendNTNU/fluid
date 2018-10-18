# Actionlib wrapper blueprint

Fluid FSM needs a way to receive an operation request, perform a task which take an arbitrary amount of time, return 
periodic feedback on the progress and respond when the operation has finished. This is where ROS actionlib comes into
the picture. It's important to keep in mind that an operation could be canceled at any time, and therefore the operation
will provide an result with the associated data for when it was canceled.

Be aware that an operation within fluid equals an action in ROS actionlib.

## Operation goal
The request a client makes when it wants the FSM to carry out an operation. It will include:
- Pose


## Operation feedback
The action server will provide feedback during the operation. It will include:
- Pose
- Some other data?


## Operation result
The action server will provide a result when the operation is complete, this is fired once. It will include:
- Final pose of the operation when it was finished/canceled
- If the operation completed or not (whether it was cancelled or not)


## Action specification
In order for the action client and the action server to communicate we need to define a specification - a few messages
on which they communicate. This is defined in an .action file in the ./action directory for the package. For a generic
move operation it will be:

```
# The operation goal
pose target_pose
---
# Operation result
pose final_pose
---
# Operation feedback
pose current_pose
```

The different types of operations for fluid would be:
- Initialize
- Take off
- Move
- Land


## ActionClient

Example of an action client for a move operation (taken from http://wiki.ros.org/actionlib)

```
#include <fluid_operations/MoveAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<fluid_operations::MoveAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_operation_client");
  Client client("move", true);
  client.waitForServer();
  fluid_actions::MoveGoal move_operation_goal;
  // move_operation_goal.pose = set_point_goal
  client.sendGoal(move_operation_goal);
  client.waitForResult(ros::Duration(5.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // Move complete
  printf("Current State: %s\n", client.getState().toString().c_str());
  return 0;
}
```


## ActionServer

Example of an action server for a move operation (taken from http://wiki.ros.org/actionlib)

```
#include <fluid_operations/MoveAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<fluid_operations::MoveAction> Server;

void execute(const fluid_operations::MoveGoalConstPtr& goal, Server* as) {
  // Execute operation within fluid FSM
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_server");
  ros::NodeHandle n;
  Server server(n, "move", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
```

Fluid FSM should generalize the server code with some sort of wrapping so it's easy to set up a server for all the 
possible operations. That wrapper has to be some sort of class which encapsulates:
- The kind of operation/action (fluid_operation::some_action)
- Generic execute based on the given operation