#include "operation_identifier.h"

std::string getPX4ModeForOperationIdentifier(const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::LAND:
            return PX4_MODE_LAND;
        default:
            return PX4_MODE_OFFBOARD;
    }
}

std::string getStringFromOperationIdentifier(const OperationIdentifier& operation_identifier) {
    switch (operation_identifier) {
        case OperationIdentifier::TAKE_OFF:
            return "TAKE_OFF";
        case OperationIdentifier::HOLD:
            return "HOLD";
        case OperationIdentifier::EXPLORE:
            return "EXPLORE";
        case OperationIdentifier::TRAVEL:
            return "TRAVEL";
        case OperationIdentifier::LAND:
            return "LAND";
        case OperationIdentifier::EXTRACT_MODULE:
            return "EXTRACT_MODULE";
        case OperationIdentifier::UNDEFINED:
            return "UNDEFINED";
    }
}
