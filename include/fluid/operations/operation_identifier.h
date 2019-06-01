//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_OPERATION_DEFINES_H
#define FLUID_FSM_OPERATION_DEFINES_H

namespace fluid {
    namespace OperationIdentifier {
        const std::string		Init           		= "init_operation",
                                TakeOff        		= "take_off_operation",
                                Move           		= "move_operation",
                                MoveOriented 		= "move_operation_oriented",
                                Land      	   		= "land_operation",
                                PositionFollow 		= "position_follow_operation";
    }
}

#endif //FLUID_FSM_OPERATION_DEFINES_H
