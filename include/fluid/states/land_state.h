#ifndef LAND_STATE_H
#define LAND_STATE_H

#include "state.h"
#include "util.h"

/** 
 *  \brief The state is relative to current position, will land at the current position no matter what is passed
 *         as the setpoint.
 */
class LandState : public State {
public:
    explicit LandState() : State(StateIdentifier::Land, PX4StateIdentifier::Land, false, false) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif
