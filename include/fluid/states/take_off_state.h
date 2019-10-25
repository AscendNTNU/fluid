#ifndef FLUID_FSM_TAKE_OFF_STATE_H
#define FLUID_FSM_TAKE_OFF_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class TakeOffState
     *  \brief Represents the state where the drone is on taking off from ground straight up. This state's reference point is the current position,
     *         so the setpoint is irrelevant. 
     */
    class TakeOffState: public State {

    private:

        geometry_msgs::Point initial_position;

    public:

        explicit TakeOffState() : State(fluid::StateIdentifier::TakeOff, fluid::PX4::Offboard, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;

        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) override;
        ControllerType getPreferredController() const override;
    };
}



#endif //FLUID_FSM_TAKE_OFF_STATE_H
