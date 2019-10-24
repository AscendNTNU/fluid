#ifndef FLUID_FSM_IDLE_STATE_H
#define FLUID_FSM_IDLE_STATE_H

#include "state.h"

namespace fluid {

    /** \class IdleState
     *  \brief Represents the state where the drone is on ground, armed and spinning its rotors
     */
    class IdleState: public State {

    private:

        const ros::Duration halt_interval_ = ros::Duration(2.0);        ///< The amoun of time to stay at idle before
                                                                        ///< saying we're finished executing this state.

        ros::Time initial_time_;                                        ///< The time the idle state starts up.
                                                                        ///< Makes it possible to halt at the 
                                                                        ///< idle state for an interval to 
                                                                        ///< make sure that we for example are stable
                                                                        ///< at ground after a land before we do 
                                                                        ///< something else.
                                                                        
    public:

        explicit IdleState() : State(StateIdentifier::Idle, PX4::Offboard, true, false) {}
        
        bool hasFinishedExecution() override;
        void initialize() override;

        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) const override;
        ControllerType getPreferredController() override;
  };
}

#endif //FLUID_FSM_IDLE_STATE_H
