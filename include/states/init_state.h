//
//  Created by Simen Gangstad on 15/10/2018.
//

#ifndef FLUID_FSM_INIT_STATE_H
#define FLUID_FSM_INIT_STATE_H

#include "../mavros/mavros_state.h"
#include "../mavros/mavros_state_setter.h"

#include <ros/ros.h>

namespace fluid {

    /** \class InitState
     *  \brief Makes sure everything is initialized (link to mavros and px4) before any further transitions are called.
     */
    class InitState: public MavrosState {

    private:

        ros::NodeHandle handle;
        bool armed = false;                                 ///< Tells whether the drone is armed or not.

    public:
        
        /** Initializes the init state.
         */
        InitState() : MavrosState("init") {}

        /**
         * Overridden function. @see State::hasFinishedExecution
         */
        bool hasFinishedExecution();

        /**
         * Overridden function. @see State::tick
         */
        void tick();

        /**
         * Overridden function. @see State::perform
         */
        void perform();
    };
}


#endif /* init_state_h */
