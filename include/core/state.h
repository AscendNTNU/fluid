#ifndef FLUID_FSM_STATE_H
#define FLUID_FSM_STATE_H

#include <memory>
#include <vector>
#include <string>


namespace fluid {
    struct Point {
        double x = 0.0, y = 0.0, z = 0.0;
    };

    struct Quaternion {
        double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
    };

    struct Pose {
        Point point;
        Quaternion quaternion;
    };

    class State;

    /** \class StateDelegate
     *  \brief Interface for callbacks from states.
     *
     *  Implement this and set the delegate in the state object to retrieve information about when the state
     *  began and finished.
     */
    class StateDelegate {
    public:
        /**
         * Gets called when the state begins acting.
         */
        virtual void stateBegan(const State& sender) = 0;

        /**
         * Gets called when the state finished acting.
         */
        virtual void stateFinished(const State& sender) = 0;
    };


    /** \class State
     *  \brief Interface for states within the finite state machine.
     *
     *  The state class is an interface which encapsulates an action, callbacks when the state started and
     *  finished as well as which states the state can transition to.
     */
    class State {
    public:

        std::weak_ptr<StateDelegate> state_delegate_p; ///< Delegation messages are sent to this object.
                                                       ///< The class of the object has to implement the StateDelegate
                                                       ///< interface.

        const std::string identifier; ///< Identifier of the state
        
        // TODO: switch to ros pose, this pose is here temporariliy for debugging purposes
        Pose pose; ///< The current pose of the state.
        
        /**
         * Initializes state with a pose.
         *
         * @param pose The pose the state should appear at.
         */
        State(std::string identifier, Pose pose): identifier(identifier), pose(pose) {}

        /**
         * Performs the logic for the given state.
         */
        virtual void perform() {}
    };
}
#endif
