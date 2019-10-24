#ifndef FLUID_FSM_MOVE_STATE_H
#define FLUID_FSM_MOVE_STATE_H

#include "state.h"

namespace fluid {

    /** \class MoveState
     *  \brief Represents the state where the drone is moving from a to b.
     */
    class MoveState: public State {

    public:

        explicit MoveState() : State(fluid::StateIdentifier::Move, fluid::PX4::Offboard, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;

        ControllerType getPreferredController() const override;
        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) const override;
   };
}

#endif 
