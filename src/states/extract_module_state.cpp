/**
 * @file extract_module_state.cpp
 */
#include "extract_module_state.h"

ExtractModuleState::ExtractModuleState() : State(StateIdentifier::ExtractModule,
                                                 PX4StateIdentifier::Offboard,
                                                 false,
                                                 false) {}

bool ExtractModuleState::hasFinishedExecution() const { return true; }

void ExtractModuleState::initialize() {}