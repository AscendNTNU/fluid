//
// Created by simengangstad on 18.10.18.
//

#include "../../include/operations/operation_identifier.h"

std::ostream& operator<<(std::ostream& os, const fluid::OperationIdentifier identifier) {
    std::string description;

    switch (identifier) {
        case fluid::OperationIdentifier::take_off:
            description = "take_off";
            break;

        case fluid::OperationIdentifier::move:
            description = "move";
            break;

        case fluid::StateIdentifier::land:
            description = "land";
            break;

        default:
            description = "unknown";
            break;
    }

    return os << description;
}