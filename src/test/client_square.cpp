#include <ros/ros.h>
#include <fluid/client.h>
#include <fluid/state_identifier.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "client_square");
    ros::NodeHandle nh;

    fluid::Client client("", 180);

    mavros_msgs::PositionTarget setpoint;
    setpoint.position.x = 0;
    setpoint.position.z = 1.0;

    client.requestTakeOff(0.8, [&](bool completed) {
	/*client.requestMove(setpoint, [&] (bool completed) {
		setpoint.position.y = 1;
		client.requestMove(setpoint, [&] (bool completed) {
			setpoint.position.x = 0;
			setpoint.position.y = 1;
			client.requestMove(setpoint, [&] (bool completed) {
				setpoint.position.x = 0;
				setpoint.position.y = 0;
				client.requestOperationToState(fluid::StateIdentifier::Land, setpoint, [&] (bool completed) {});
			});
		});
	});*/
    });


    ros::Rate rate(1);

    while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
    }

    return 0;
}
