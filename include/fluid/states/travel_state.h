#ifndef TRAVEL_STATE_H
#define TRAVEL_STATE_H

#include "move_state.h"
#include "state_identifier.h"

class TravelState : public MoveState {

public:
    TravelState() : MoveState(StateIdentifier::Travel, 1, .5, .4) {}
};

#endif