# Fluid FSM

## Run instructions for gazebo simulator

1. Clone fluid into your catkin workspace in the src-folder.
2. Run `source devel/setup.bash` and `catkin build` at root in the catkin workspace.
3. Start mavros: `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
4. Start gazebo from your gazebo firmware folder: `make posix_sitl_default gazebo`
5. Start fluid FSM: `rosrun fluid_fsm main`
6. Start test client: `rosrun fluid_fsm test_client`


The behavioral code for the drone under the simulation is found at `src/test_client.cpp`.
