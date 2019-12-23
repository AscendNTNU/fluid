#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "state.h"

namespace fluid {

    class MoveState: public State {

    private:
        const double position_threshold;
        const double velocity_threshold;
        const double speed;
        bool been_to_all_points = false;
        std::vector<geometry_msgs::Point>::iterator current_destination_point_iterator;

    protected:
        explicit MoveState(StateIdentifier state_identifier, double speed, double position_threshold, double velocity_threshold) : 
        State(state_identifier, PX4StateIdentifier::Offboard, false, true), 
        speed(speed),
        position_threshold(position_threshold),
        velocity_threshold(velocity_threshold) {}

    public:

        bool hasFinishedExecution() const override;
        virtual void tick() override;
        virtual void initialize() override;
   };
}

#endif 
