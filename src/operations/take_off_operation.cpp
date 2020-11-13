/**
 * @file take_off_operation.cpp
 */

#include "take_off_operation.h"

#include "fluid.h"
#include "mavros_interface.h"
#include "util.h"

TakeOffOperation::TakeOffOperation(float height_setpoint)
    : Operation(OperationIdentifier::TAKE_OFF, false), height_setpoint(height_setpoint) {}

bool TakeOffOperation::hasFinishedExecution() const {
    const float distance_threshold = Fluid::getInstance().configuration.distance_completion_threshold;
    const float velocity_threshold = Fluid::getInstance().configuration.velocity_completion_threshold;
    return Util::distanceBetween(getCurrentPose().pose.position, setpoint.position) < 0.1 &&
           std::abs(getCurrentTwist().twist.linear.x) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.y) < velocity_threshold &&
           std::abs(getCurrentTwist().twist.linear.z) < velocity_threshold;
}

void TakeOffOperation::initialize() {
    //trying to copy the python script made by simen: https://github.com/AscendNTNU/ascend_simulator/blob/main/src/takeoff.py
/*    ros::Rate rate(30);
    setpoint.type_mask = TypeMask::POSITION;
    setpoint.position.x = 0;
    setpoint.position.y = 0;
    setpoint.position.z = 0;
    setpoint.yaw = getCurrentYaw();
    setpoint.coordinate_frame = 1;
    setpoint.header.frame_id = "map";

    for (int i=0; i<100;i++)
    {
        setpoint.header.stamp = ros::Time::now();
        publishSetpoint();
        rate.sleep();
    }
    */
    MavrosInterface mavros_interface;
    mavros_interface.establishContactToArduPilot();
    Fluid::getInstance().getStatusPublisherPtr()->status.linked_with_px4 = 1;

    mavros_interface.requestArm(Fluid::getInstance().configuration.should_auto_arm);
    Fluid::getInstance().getStatusPublisherPtr()->status.armed = 1;

    mavros_interface.requestOffboard(Fluid::getInstance().configuration.should_auto_offboard);
    Fluid::getInstance().getStatusPublisherPtr()->status.px4_mode = PX4_MODE_OFFBOARD;

//    setpoint.header.stamp = ros::Time::now();
//    publishSetpoint();
//    rate.sleep();

    //send take off command
    //setpoint.position.z = 2;
    //mavros_interface.requestTakeOff(setpoint);

/*    
    // Spin until we retrieve the first pose
    while(ros::ok() && getCurrentPose().header.seq == 0) {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "publish setPoint for takeoff\n");
        publishSetpoint();
        ros::spinOnce();
    }
*/
}
