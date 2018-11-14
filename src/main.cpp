//
// Created by simengangstad on 27.09.18.
//

#include "../include/core/operation/operation.h"
#include <ros/ros.h>
#include <list>
#include <fluid_fsm/OperationGoal.h>
#include <fluid_fsm/OperationAction.h>
#include "actionlib/operation_server.h"

#include "../include/states/init_state.h"
#include "../include/states/take_off_state.h"
#include "../include/states/hold_state.h"
#include "../include/core/state.h"
#include "../include/core/transition.h"

#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "fluid_fsm");

    fluid::OperationServer operation_server;
    operation_server.execute();

    return 0;
}
