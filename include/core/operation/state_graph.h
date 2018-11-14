//
// Created by simengangstad on 11.10.18.
//

#ifndef FLUID_FSM_STATE_GRAPH_H
#define FLUID_FSM_STATE_GRAPH_H

#include "graph.h"

#include <ros/ros.h>

namespace fluid {
    /** \class StateGraph
     *  \brief Represents a graph with all of the states in fluid_fsm
     */
    class StateGraph: public Graph {

    private:

        ros::NodeHandlePtr node_handle_p;                            ///< Used for initializing states and their
                                                                     ///< respective ros pubilshers and subscribers


        bool initialized_ = false;

    public:

        std::shared_ptr<State> current_state_p;                      ///< The current state of the state graph

        bool isInitialized();

        /**
         * Initializes the state graph with a set of states.
         */
        void initialize();

        /**
         * @return ROS node handle pointer, used when states need to do something specific with ROS.
         */
        ros::NodeHandlePtr getNodeHandlePtr();
    };
}

#endif //FLUID_FSM_STATE_GRAPH_H
