/**
 * @file extract_module_operation.h
 */

#ifndef EXTRACT_MODULE_OPERATION_H
#define EXTRACT_MODULE_OPERATION_H

#include "operation.h"
#include "operation_identifier.h"

/**
 * @brief Represents the operation where the drone is extracting the module.
 */
class ExtractModuleOperation : public Operation {
   public:
    /**
     * @brief Sets up the extract moduel operation.
     *
     */
    explicit ExtractModuleOperation();

    /**
     * @return true When the module has been extracted.
     */
    bool hasFinishedExecution() const override;

    /**
     * @brief Initializes the extract module operation.
     */
    void initialize() override;
};

#endif
