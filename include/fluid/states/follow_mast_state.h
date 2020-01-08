#ifndef FOLLOW_MAST_STATE_H 
#define FOLLOW_MAST_STATE_H 

#include "state.h"
#include "state_identifier.h"

class FollowMastState : public State {

public:
    explicit FollowMastState() : State(StateIdentifier::FollowMast, PX4StateIdentifier::Offboard, false, false) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif


