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



# Writing clients

Fluid FSM is built around a client-server architecture, where a client requests the server to do something. In other words, we (the client) ask the drone (the server) to do **operations** which consist of a series of *states*. 

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

## Example

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
