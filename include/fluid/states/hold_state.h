#ifndef HOLD_STATE_H
#define HOLD_STATE_H

#include "state.h"
#include "util.h"

/** 
 *  \brief Drone is hovering at a certain altitude
 */
class HoldState : public State {
public:
    explicit HoldState() : State(StateIdentifier::Hold, PX4StateIdentifier::Offboard, true, false) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif
