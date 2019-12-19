#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "state.h"

namespace fluid {

    /** \class MoveState
     *  \brief Represents the state where the drone is moving from a to b.
     */
    class MoveState: public State {

    private:

        bool been_to_all_points = false;
        std::vector<geometry_msgs::Point>::iterator current_destination_point_iterator;

    public:

        explicit MoveState() : State(fluid::StateIdentifier::Move, fluid::PX4::Offboard, false, true, false) {}

        bool hasFinishedExecution() const override;
        void tick() override;
        void initialize() override;
   };
}

#endif 
