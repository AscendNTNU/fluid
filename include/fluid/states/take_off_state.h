#ifndef TAKE_OFF_STATE_H
#define TAKE_OFF_STATE_H

#include "state.h"
#include "util.h"

/** 
 *  \brief Will take off at the current position and not move after, in other words, ignores the setpoint x and y 
 *         values. 
 */
class TakeOffState : public State {
public:
    explicit TakeOffState() : State(StateIdentifier::TakeOff, PX4StateIdentifier::Offboard, false, true) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif
