//
// Created by simengangstad on 08.11.18.
//

#include "../../include/actionlib/operation_server.h"
#include "../../include/operations/operation_defines.h"
#include "operations/init_operation.h"
#include "operations/move_operation.h"
#include "operations/land_operation.h"
#include "operations/take_off_operation.h"
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/String.h>
#include <fluid_fsm/OperationGoal.h>


void fluid::OperationServer::execute(const fluid_fsm::OperationGoalConstPtr &goal) {

    mavros_msgs::PositionTarget position_target;
    position_target.position = goal->target_pose.position;
    const std_msgs::String operation_identifier = goal->type;

    ROS_INFO_STREAM("Got operation request for " << operation_identifier);

    if (operation_identifier.data == fluid::operation_identifiers::INIT) {

    }
    else if (operation_identifier.data == fluid::operation_identifiers::IDLE) {

    }
    else if (operation_identifier.data == fluid::operation_identifiers::TAKE_OFF) {

    }
    else if (operation_identifier.data == fluid::operation_identifiers::HOLD) {

    }
    else if (operation_identifier.data == fluid::operation_identifiers::MOVE) {

    }
    else if (operation_identifier.data == fluid::operation_identifiers::LAND) {

    }

    /*
    switch (operation_identifier) {
        case fluid::OperationIdentifier::init: {
            std::shared_ptr<fluid::InitOperation> init_operation = std::make_shared<fluid::InitOperation>(position_target);
            operationRequestedCallback(init_operation);
            break;
        }

        case fluid::OperationIdentifier::move: {
            std::shared_ptr<fluid::MoveOperation> move_operation = std::make_shared<fluid::MoveOperation>(position_target);
            operationRequestedCallback(move_operation);
            break;
        }

        case fluid::OperationIdentifier::land: {
            std::shared_ptr<fluid::LandOperation> land_operation = std::make_shared<fluid::LandOperation>(position_target);
            operationRequestedCallback(land_operation);
            break;
        }

        case fluid::OperationIdentifier::take_off: {
            std::shared_ptr<fluid::TakeOffOperation> take_off_operation = std::make_shared<fluid::TakeOffOperation>(position_target);
            operationRequestedCallback(take_off_operation);
            break;
        }
    }
*/
    actionlib_action_server_.setSucceeded();
}