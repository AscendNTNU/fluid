//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"
#include <ros/ros.h>
#include <fluid_fsm/MoveGoal.h>
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/LandAction.h>
#include <fluid_fsm/LandGoal.h>
#include "actionlib/operation_server.h"
#include "../include/operations/operation_identifier.h"

#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv) {

    /*
    std::shared_ptr<Operation> operation;

    operation = std::make_shared<Operation>();

    operation->addState(std::make_shared<MoveState>(pose));
    operation->addState(std::make_shared<IdleState>(pose));

    op eration->perform();
*/
/*
    ros::init(argc, argv, "fluid_fsm");
    fluid::OperationServer<fluid_fsm::MoveAction, fluid_fsm::MoveGoalConstPtr> move_server(fluid::OperationIdentifier::move);
    fluid::OperationServer<fluid_fsm::LandAction, fluid_fsm::LandGoalConstPtr> land_server(fluid::OperationIdentifier::land);
    ros::spin();
*/


    return 0;
}
