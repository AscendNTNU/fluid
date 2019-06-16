# Fluid FSM


# Installation and use

## What you need

1. ROS Kinetic: http://wiki.ros.org/kinetic/Installation 
2. MAVROS: https://dev.px4.io/en/ros/mavros_installation.html
3. Control simulator (or plain Gazebo with PX4): https://github.com/AscendNTNU/control_simulator

## Run instructions for gazebo simulator

1. Make sure you have MAVROS installed and PX4 and gazebo built. 
2. Clone fluid into your catkin workspace in the src-folder.
3. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
4. Start control_simulator. See documentation here: https://github.com/AscendNTNU/control_simulator
5. Export the namespace `export ROS_NAMESPACE=your_namespace`.
6. Start fluid server: `roslaunch fluid square_gazebo.launch`.
7. Start your client. Remember to construct the client with the same namespac.

There are some examples in the `src/test` folder.

If you're running control_simulator you have to wait until EKF vision fuse is set up. Read more here:
https://confluence.ascendntnu.no/pages/viewpage.action?pageId=21955671


## Run instructions for physical drone with Pixhawk flight controller

1. Clone fluid into the catkin workspace on the drone. 
2. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
3. Optional: Run `export ROS_NAMESPACE=your_namespace` (if you want to launch the server in a namespace).
4. Start fluid server via the roslaunch file: `roslaunch fluid_fsm server_pixhawk.launch`.
5. Launch your client node.


# Architecture

Fluid FSM is built around a client-server architecture, where a client requests the server to do something. In other words, we (the client) ask the drone (the server) to do **operations** which consist of a series of *states*. 

It also relies on a graph data structure where all the states in the finite state machine lives. This makes it possible to find the path from any
state to another easy by traversing through the graph. In that way, if we want to move to a certain point, the FSM knows that it has to initialize,
take off, rotate in the movement direction before it moves. This makes the FSM very flexible and reduces complexity on the client side. 

## Different states

The different states are:
- Init
- Take off
- Hold (hover)
- Move
- Rotate
- Land
- Position follow (follow a given position, can be a person or an object)
    - In the position follow state the the drone will follow a position given on a topic. If there isn't anything
      published on that topic it will just hover.

The drone will do certain states relative to the current position. E.g. init, take off, land and rotate. So if you request 
the drone to move to a certain position from ground, it will take off from the current position rotate so it's facing the
direction of the movement and then move. 

## Flow

The flow consist of: 

1. The client issuing that we want to go to a certain state
2. The server receiving the request and setting up an operation to that state 
3. The operation figuring out how to get to that state from the current state (a list of states)
4. The operation traversing through each state in that list and executing the respective state logic and transitioning to the next state in the list
5. The operation transitioning to a *steady* state after we've finished executing each state. The steady state is a state which, as its name implies,
   can be executed indefintely, e.g. hold (hover) or idle (idle at ground). This makes sure that we're ready for the next operation.
6. The operation telling the server that it's done.
7. The server receiving that message and telling the client that the operation has finished.

If the operation failed somewhere along the path (if for example another operation was issued), the server will notify the client that the operation
failed.


# Writing clients - example

```cpp
#include <ros/ros.h>
#include <fluid/client.h>

int main(int argc, char** argv) {

    // Initialize the ROS node.
    ros::init(argc, argv, "client");

    // Set up a client with a given namespace and a timeout value. This makes it possible for us to issue operations
    // to the drone. The namespace is the namespace which the server is running in. This can be left out here, but is
    // given as an option in order to create multiple clients controlling multiple drones.
    fluid::Client client("drone_1");

    // We set a setpoint for the drone to move to.
    mavros_msgs::PositionTarget setpoint;
    setpoint.position.x = 5.0;

    // We issue the drone to move from the current state. If the drone e.g. is at ground, it will go through /
    // initialization, take off and then move. 
    // 
    // Have a look at client.h for all the different commands. You can also request the FSM to transition to a specific
    // state from the current state with the function requestOperationToState. 
    client.requestMove(setpoint, [&](bool completed) {
        ROS_INFO_STREAM("Operation finished with completion: " << completed);
    });
 
    // Since the above function calls are asynchronous, we make ros take over the main loop so the node doesn't 
    // get killed.
    ros::spin();

    return 0;
}
```
