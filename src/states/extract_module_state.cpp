/**
 * @file extract_module_state.cpp
 */
#include "extract_module_state.h"

ExtractModuleState::ExtractModuleState() : State(StateIdentifier::EXTRACT_MODULE,
                                                 false,
                                                 false) {}

bool ExtractModuleState::hasFinishedExecution() const { return true; }

void ExtractModuleState::initialize() {}