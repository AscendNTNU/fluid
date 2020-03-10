/**
 * @file extract_module_state.h
 */

#ifndef EXTRACT_MODULE_STATE_H
#define EXTRACT_MODULE_STATE_H

#include "state.h"
#include "state_identifier.h"

/**
 * @brief Represents the state where the drone is extracting the module.
 */
class ExtractModuleState : public State {
   public:
    /**
     * @brief Sets up the extract moduel state.
     * 
     */
    explicit ExtractModuleState();

    /**
     * @return true When the module has been extracted.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Initializes the extract module state.
     */
    void initialize() override;
};

#endif
