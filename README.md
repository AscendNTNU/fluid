# Fluid FSM

## Run instructions for gazebo simulator

Make sure you have mavros installed and PX4 and gazebo built. You also need ascend_msgs.

1. Clone fluid into your catkin workspace in the src-folder.
2. Run `source devel/setup.bash` and `catkin_make` at root of the catkin workspace.
3. Run `export ROS_IP=localhost`
4. Start gazebo from your gazebo firmware folder: `make posix_sitl_default gazebo`
5. Start fluid and test client via the roslaunch file: `roslaunch fluid_fsm client_square.launch`

The behavioral code for the simulation is found at `src/test_client.cpp`.
