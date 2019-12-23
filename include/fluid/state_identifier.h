//
// Created by simengangstad on 08.11.18.
//

#ifndef FLUID_FSM_STATE_DEFINES_H
#define FLUID_FSM_STATE_DEFINES_H

#include <string>
#include <map>

namespace fluid {

    enum class StateIdentifier {
        // Null is here to make the bredth first search possible in state graph
        Init, Idle, TakeOff, Hold, Exploration, Travelling, Land, Rotate, Null

    };

    const std::map<StateIdentifier, std::string> StateIdentifierStringMap = {
        {StateIdentifier::Init,             "init"},
        {StateIdentifier::Idle,             "idle"},
        {StateIdentifier::TakeOff,          "take_off"},
        {StateIdentifier::Hold,             "hold"},
        {StateIdentifier::Exploration,      "exploration"},
        {StateIdentifier::Travelling,       "travelling"},
        {StateIdentifier::Land,             "land"},
        {StateIdentifier::Rotate,           "rotate"}
    };

    enum class PX4StateIdentifier {
        Offboard, Land
    };

    const std::map<PX4StateIdentifier, std::string> PX4StateIdentifierStringMap = {
        {PX4StateIdentifier::Offboard,      "OFFBOARD"}, 
        {PX4StateIdentifier::Land,          "AUTO.LAND"},
    };
}

#endif 
