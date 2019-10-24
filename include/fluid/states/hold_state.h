#ifndef FLUID_FSM_HOLD_STATE_H
#define FLUID_FSM_HOLD_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class HoldState
     *  \brief Represents the state when the drone is hovering at a certain altitude
     */
    class HoldState: public State {

    private:
        geometry_msgs::Point initial_position;

    public:

        explicit HoldState() : State(StateIdentifier::Hold, PX4::Offboard, true, false) {}

        bool hasFinishedExecution() override;
        void initialize() override;

        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) const override;
        ControllerType getPreferredController() override;
    };
}

#endif //FLUID_FSM_HOLD_STATE_H
