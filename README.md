# Fluid FSM

## Run instructions for gazebo simulator

Make sure you have mavros installed and PX4 and gazebo built 

1. Clone fluid into your catkin workspace in the src-folder.
2. Run `source devel/setup.bash` and `catkin build` at root of the catkin workspace.
3. Start gazebo from your gazebo firmware folder: `make posix_sitl_default gazebo`
4. Start fluid and test client via the roslaunch file: `roslaunch fluid_fsm test_square.launch`.

The behavioral code for the simulation is found at `src/test_client.cpp`.
