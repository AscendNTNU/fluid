# Fluid FSM


# Installation and use

## What you need

1. Ros kinetic: http://wiki.ros.org/kinetic/Installation 
2. Mavros: https://dev.px4.io/en/ros/mavros_installation.html
3. Gazebo: https://dev.px4.io/en/setup/dev_env_linux.html#jmavsimgazebo-simulation

## Run instructions for gazebo simulator

1. Make sure you have mavros installed and PX4 and gazebo built. 
2. Clone fluid into your catkin workspace in the src-folder.
3. Run `source devel/setup.bash` and `catkin_make` at root of the catkin workspace.
4. Run `export ROS_IP=localhost`.
5. Start gazebo from your gazebo firmware folder: `make posix_sitl_default gazebo`.
6. Start fluid and test client via the roslaunch file: `roslaunch fluid_fsm client_square.launch`.

The behavioral code for the simulation is found at `src/test_client.cpp`.

### Running only the server

If you want to only start the server and write your own client, just run `roslaunch fluid_fsm server_gazebo.launch` and then start your own client node. 


## Run instructions for physical drone with Pixhawk flight controller

1. Clone fluid into the catkin workspace on the drone. 
2. Run `source devel/setup.bash` and `catkin_make` at root of the catkin workspace.
3. Start fluid server via the roslaunch file: `roslaunch fluid_fsm server_pixhawk.launch`.
4. Launch your client node.



# Writing clients

Fluid FSM is built around a client-server architecture, where a client requests the server to do something. In other words, we (the client) ask the drone(the server) to do operations. 

In order to initialize the drone and link it up with PX4 and arm it we need to request that from it with the following code: 

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include "../include/actionlib/operation_client.h"
#include "../include/operations/operation_identifier.h"

int main(int argc, char** argv) {

    // Initialize the ROS node.
    ros::init(argc, argv, "test_client");
    ros::NodeHandle nh;

    // Initialize an origin pose
    geometry_msgs::Pose pose;

    bool initialized = false;

    // An operation client is the object which is going to send requests to the drone to do things, like moving, 
    // taking off, landing, initializing.
    //
    // In this case we set up an initialize operation client with an argument of 20. The argument states how long
    // the operation client will wait for a response from the drone. If the drone hasn't been able to initialize and
    // respond within 20 seconds, we move on. Notice that the type is a generic operation client, that is because
    // we can use this client for all the different things we want our drone to do. But just for the clarity of this
    // example, we name it init_operation_client. 
    fluid::OperationClient init_operation_client(20);
    
    // We request an initialize operation from the drone. Since every operation client is generic we have to specify 
    // which operation we want to do. In this case it is an INIT operation since we want the drone to initialize. 
    //
    // The second argument is the pose we want the drone to do the operation at. In the case of an init operation it's
    // insignificant, since it will initialize at the current position it is at anyways. But in the case of a move
    // operation, this pose would be the place you want the drone to fly to.
    //
    // The third argument is a lambda. This lambda will get called when the operation finished and will include a
    // flag whether it completed or not.  
    //
    // This function call is asynchronous, so we have to "wait" until it finishes because the drone has to be 
    // initialized in order to do other things.
    init_operation_client.requestOperation(fluid::OperationIdentifier::Init, pose, [&](bool completed) {
        initialized = completed;
    });

    // We set up a ros loop which will wait until we have initialized
    ros::Rate wait_rate(20);
    while (ros::ok() && !initialized) {
        ros::spinOnce();
        wait_rate.sleep();
    }





    // Now we want to take off. We set up an operation client:
    fluid::OperationClient operation_client(10);

    // And we want the drone to take off so it's 1.5 meters over ground:
    pose.position.z = 1.5;

    // We then request take off:
    operation_client.requestOperation(fluid::OperationIdentifier::TakeOff, pose, [&](bool completed) {
        if (completed) {

            ROS_INFO("Drone completed take off!");

            // We completed take off and want to fly in a straight line forward
            pose.position.x = 1;
            pose.position.z = 1.5;

            operation_client.requestOperation(fluid::OperationIdentifier::Move, pose, [&](bool completed) {
                if (completed) {
                    ROS_INFO("Drone flew to the position!");
                }
            });
        }
    });
  
    // Since the above function calls are asynchronous, we make ros take over the main loop so the node doesn't 
    // get killed.
    ros::spin();

    return 0;
}
```


## Different operations

The different operations are:

- Init
- Take off
- Move
- Land
- Position follow (follow a given position, can be a person or an object)
    - In the position follow state the the drone will follow a position given on a topic. If there isn't anything
      published on that topic it will just hover.
      