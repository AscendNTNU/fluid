//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_HOLD_STATE_H
#define FLUID_FSM_HOLD_STATE_H

#include "../core/state.h"

/** \class HoldState
 *  \brief Keeps the drone hovering at a certain altitude
 */
class HoldState: public State {

public:

    /**
     * Initializes the hold state with a pose.
     *
     * @param pose The pose the drone should be hovering at.
     */
    HoldState(Pose pose) : State("hold", pose) {}

    /**
     * Publishes a stream of set points (the pose of the position hold state) to PX4.
     */
    void perform();
};

#endif //FLUID_FSM_HOLD_STATE_H
