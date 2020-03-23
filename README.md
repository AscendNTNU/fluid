# README 

## Installation 

### What you need

1. [ROS Melodic](http://wiki.ros.org/melodic/Installation) 
2. [MAVROS](https://github.com/mavlink/mavros)
3. [AirSim (or plain Gazebo with PX4)](https://microsoft.github.io/AirSim/)

## Architecture

Fluid FSM is built around a client-server architecture with ROS services, where a client requests the server to do something through ROS services. In other words, we (the client) ask the drone (the server) to do **operations**. 

Have a look at [Flow](Flow.md) and [Operations](Operations.md) for more details.

## Use

### Run instructions for simulator

1. Make sure you have MAVROS installed and PX4 and gazebo built. 
2. Clone fluid into your catkin workspace in the src-folder.
3. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
4. Start Airsim. 
5. Start fluid server: `roslaunch fluid simulator.launch`.
6. Start your client. 

There are some examples in the `src/examples` folder.


### Run instructions for physical drone with Pixhawk flight controller

1. Clone fluid into the catkin workspace on the drone. 
2. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
3. Start fluid server via the roslaunch file: `roslaunch fluid pixhawk.launch`.
4. Launch your client node.

## Writing clients

You have to use ROS services in order to communicate with the state machine. Have a look at the python and C++ examples in the [src/examples](https://github.com/AscendNTNU/fluid/tree/master/src/examples) folder.

