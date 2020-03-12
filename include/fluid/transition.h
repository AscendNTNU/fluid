#ifndef TRANSITION_H
#define TRANSITION_H

#include <ros/ros.h>
#include <memory>

#include "mavros_interface.h"
#include "state.h"

/** 
 *  \brief Handles state changes and the communication with the FSM in PX4.
 */
class Transition {
   private:
    MavrosInterface mavros_interface;  ///< Used to set states within the Pixhawk.

   public:
    const std::shared_ptr<State> source_state_ptr, destination_state_ptr;
    Transition(std::shared_ptr<State> source_state_ptr, std::shared_ptr<State> destination_state_ptr);

    /**
     * Performs the transition between the source state and the destination state.
     */
    void perform();
};

#endif
