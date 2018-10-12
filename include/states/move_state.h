//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "../core/state.h"

namespace fluid {
    /** \class MoveState
     *  \brief Represents the state where the drone is moving from a to b.
     */
    class MoveState: public State {
    private:
        const Pose set_point_; ///< The pose the drone will be moving to in the move state.

    public:

        /** Initializes the move state with a current pose and a set point pose.
         *
         * @param pose Initial pose
         * @param set_point Set point pose
         */
        // TOOD: Change set_point to a more suitable variable?
        MoveState(Pose pose, Pose set_point) : State("move", pose), set_point_(set_point) {}

        /**
         * Publishes a stream of set points (which is the set_point of the move state) to PX4.
         */
        void perform();
    };
}

#endif //FLUID_FSM_MOVE_STATE_H
