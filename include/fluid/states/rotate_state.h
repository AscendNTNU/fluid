#ifndef ROTATE_STATE_H
#define ROTATE_STATE_H

#include "state.h"
#include "util.h"

class RotateState : public State {
public:
    explicit RotateState() : State(StateIdentifier::Rotate, PX4StateIdentifier::Offboard, false, true) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif
