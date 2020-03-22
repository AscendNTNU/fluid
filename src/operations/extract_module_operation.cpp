/**
 * @file extract_module_operation.cpp
 */
#include "extract_module_operation.h"

ExtractModuleOperation::ExtractModuleOperation() : Operation(OperationIdentifier::EXTRACT_MODULE, false) {}

bool ExtractModuleOperation::hasFinishedExecution() const { return true; }

void ExtractModuleOperation::initialize() {}