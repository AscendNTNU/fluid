//
// Created by simengangstad on 24.10.18.
//

#ifndef FLUID_FSM_OPERATION_UTIL_H
#define FLUID_FSM_OPERATION_UTIL_H

#include <operation_identifier.h>

namespace fluid {

    /** \class OperationUtil
     *  \brief Provide helper functions for operations and operation identifiers
     */
    class OperationUtil {

        /**
         * Gets a description of the operation identifier.
         *
         * @param identifier The operation identifier to get the description from.
         * @return The description of the operation identifier.
         */
        static std::string descriptionFromOperationIdentifier(fluid::OperationIdentifier identifier) {
            std::string description;

            switch (identifier) {
                case fluid::OperationIdentifier::take_off:
                    description = "take_off";
                    break;

                case fluid::OperationIdentifier::move:
                    description = "move";
                    break;

                case fluid::OperationIdentifier::land:
                    description = "land";
                    break;

                default:
                    description = "unknown";
                    break;
            }

            return description;
        }
    };
}


#endif //FLUID_FSM_OPERATION_UTIL_H
