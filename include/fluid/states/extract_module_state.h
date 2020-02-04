#ifndef EXTRACT_MODULE_STATE_H 
#define EXTRACT_MODULE_STATE_H 

#include "state.h"
#include "state_identifier.h"

class ExtractModuleState : public State {

public:
    explicit ExtractModuleState() : State(StateIdentifier::ExtractModule, PX4StateIdentifier::Offboard, false, false) {}

    bool hasFinishedExecution() const override;
    void initialize() override;
};

#endif

