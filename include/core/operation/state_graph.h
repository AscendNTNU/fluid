//
//  Created by Simen Gangstad on 24/01/2019.
//

#ifndef FLUID_FSM_STATE_GRAPH_H
#define FLUID_FSM_STATE_GRAPH_H

#include "graph.h"
#include <memory>
#include <vector>

namespace fluid {

    /** \class StateGraph
     * 
     *  \brief Represents a graph with states.
     */
    class StateGraph: public fluid::Graph {
    private:

        ros::NodeHandlePtr node_handle_p;                      ///< Used for initializing states and their
                                                               ///< respective ros pubilshers and subscribers.


        bool is_configured_ = false;                            

    public:

        std::shared_ptr<State> current_state_p;                ///< The current state of the state graph.

        /**
         * Initializes the state graph.
         */
        StateGraph() : Graph() {}

        /**
         * @return A flag which deteremines if the graph has been configured with the necessary states.
         */
        bool isConfigured();

        /**
         * Set ups the state graph with a set of states.
         * 
         * @param refresh_rate The refresh rate the states should be initialized to operate at.
         */
        void configure(unsigned int refresh_rate);

        /**
         * Checks if the start state and the end state is connected using a breadth first search and
         * returns the shortest path between them.
         *
         * @param start_state_identifier The identifier of the start state we begin at.
         * @param end_state_identifier The identifier of the end state we want to transition to.
         * 
         * @return Vector of states one has to transition to in order to get to the end state.
         */
        std::vector<std::shared_ptr<fluid::State>> getPlanToEndState(std::string start_state_identifier,
                                                                     std::string end_state_identifier);

        /**
         * @return The state with the given identifier in the graph. Will return a nullptr if not found.
         */
        std::shared_ptr<fluid::State> getStateWithIdentifier(std::string identifier);

        /**
         * @return ROS node handle pointer, used when states need to do something specific with ROS.
         */
        ros::NodeHandlePtr getNodeHandlePtr();
    };
}

#endif /* FLUID_FSM_STATE_GRAPH_H */
