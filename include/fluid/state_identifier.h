
#ifndef STATE_DEFINES_H
#define STATE_DEFINES_H

#include <map>
#include <string>

enum class StateIdentifier {
    // Null is here to make the bredth first search possible in state graph
    Init,
    Idle,
    TakeOff,
    Hold,
    Explore,
    Travel,
    Land,
    Rotate,
    ExtractModule,
    FollowMast,
    Null
};

const std::map<StateIdentifier, std::string> StateIdentifierStringMap = {
    {StateIdentifier::Init, "init"},
    {StateIdentifier::Idle, "idle"},
    {StateIdentifier::TakeOff, "take_off"},
    {StateIdentifier::Hold, "hold"},
    {StateIdentifier::Explore, "explore"},
    {StateIdentifier::Travel, "travel"},
    {StateIdentifier::Land, "land"},
    {StateIdentifier::Rotate, "rotate"},
    {StateIdentifier::ExtractModule, "extract_module"},
    {StateIdentifier::FollowMast, "follow_mast"},
    };

enum class PX4StateIdentifier {
    Offboard,
    Land
};

const std::map<PX4StateIdentifier, std::string> PX4StateIdentifierStringMap = {
    {PX4StateIdentifier::Offboard, "OFFBOARD"},
    {PX4StateIdentifier::Land, "AUTO.LAND"},
};

#endif
