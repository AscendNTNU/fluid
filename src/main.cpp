//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"
#include <ros/ros.h>
#include <fluid_fsm/MoveGoal.h>
#include <fluid_fsm/MoveAction.h>
#include "../include/actionlib/action_server.h"

int main(int argc, char** argv) {

    /*
    std::shared_ptr<Operation> operation;

    operation = std::make_shared<Operation>();

    operation->addState(std::make_shared<MoveState>(pose));
    operation->addState(std::make_shared<IdleState>(pose));

    operation->perform();
*/


    fluid::ActionServer<fluid_fsm::MoveGoalConstPtr, fluid_fsm::MoveAction> actionServer(fluid::OperationIdentifier::take_off);

    ros::init(argc, argv, "fluid_fsm");
    ros::start();

    ROS_INFO_STREAM("Hello world");

    ros::spin();

    return 0;
}
