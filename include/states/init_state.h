//
//  Created by Simen Gangstad on 15/10/2018.
//

#ifndef FLUID_FSM_INIT_STATE_H
#define FLUID_FSM_INIT_STATE_H

#include "../core/state.h"

namespace fluid {
    /** \class InitState
     *  \brief Makes sure everything is initialized (link to mavros and px4) before any further transitions are called.
     */
    class InitState: public State {
    public:
        
        /** Initializes the init state with a pose.
         *
         * @param pose The pose for the init state.
         */
        InitState(Pose pose) : State(fluid::StateIdentifier::init, pose) {}
        
        /**
         * Performs the operation of intializing all required links.
         */
        void perform();
    };
}


#endif /* init_state_h */
