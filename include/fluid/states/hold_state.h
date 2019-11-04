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

        explicit HoldState() : State(StateIdentifier::Hold, PX4::Offboard, true, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;

        std::vector<ascend_msgs::Spline> getSplinesForPath(const std::vector<geometry_msgs::Point>& path) override;
        ControllerType getPreferredController() const override;
    };
}

#endif 
