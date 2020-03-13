#include "state_identifier.h"

std::string getPX4ModeForStateIdentifier(const StateIdentifier& state_identifier) {
    switch (state_identifier) {
        case StateIdentifier::LAND:
            return PX4_MODE_LAND;
        default:
            return PX4_MODE_OFFBOARD;
    }
}

std::string getStringFromStateIdentifier(const StateIdentifier& state_identifier) {
    switch (state_identifier) {
        case StateIdentifier::TAKE_OFF:
            return "TAKE_OFF";
        case StateIdentifier::HOLD:
            return "HOLD";
        case StateIdentifier::EXPLORE:
            return "EXPLORE";
        case StateIdentifier::TRAVEL:
            return "TRAVEL";
        case StateIdentifier::LAND:
            return "LAND";
        case StateIdentifier::EXTRACT_MODULE:
            return "EXTRACT_MODULE";
        case StateIdentifier::FOLLOW_MAST:
            return "FOLLOW_MAST";
        case StateIdentifier::UNDEFINED:
            return "UNDEFINED";
    }
}
