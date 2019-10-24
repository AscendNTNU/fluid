#ifndef FLUID_FSM_INIT_STATE_H
#define FLUID_FSM_INIT_STATE_H

#include "state.h"
#include "controller.h"

namespace fluid {

    /** \class InitState
     *  \brief Makes sure everything is initialized (link to mavros and px4) before any further transitions are called.
     */
    class InitState: public State {

    private:

        bool initialized = false;                                       

    public:
        
        explicit InitState() : State(fluid::StateIdentifier::Init, fluid::PX4::Offboard, false, false) {}

        bool hasFinishedExecution() const override;

        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) const override;
        ControllerType getPreferredController() const override;

        void perform(std::function<bool (void)> tick, bool should_halt_if_steady) override;
    };
}


#endif /* init_state_h */
