//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"
#include <ros/ros.h>
#include <fluid_fsm/MoveGoal.h>
#include <fluid_fsm/MoveAction.h>
#include <fluid_fsm/LandAction.h>
#include <fluid_fsm/LandGoal.h>
#include "../include/actionlib/action_server.h"
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

    ros::init(argc, argv, "fluid_fsm");
    fluid::ActionServer<fluid_fsm::MoveGoalConstPtr, fluid_fsm::MoveAction> move_server(fluid::OperationIdentifier::move);
    fluid::ActionServer<fluid_fsm::LandGoalConstPtr, fluid_fsm::LandAction> land_server(fluid::OperationIdentifier::land);
    ros::spin();



    return 0;
}
