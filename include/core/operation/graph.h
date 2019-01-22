//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_GRAPH_H
#define FLUID_FSM_GRAPH_H

#include <memory>
#include <vector>
#include <map>
#include "../state.h"
#include "edge.h"

namespace fluid {
    typedef std::shared_ptr<State> Node;
    typedef std::map<std::string, std::vector<Node>> AdjacencyList;

    /** \class Graph
     *  \brief Represents the graph of states and transitions.
     */
    class Graph {
    private:

        std::unique_ptr<AdjacencyList> adjacency_list;         ///< Vector containing all the connections in the graph

        std::vector<std::shared_ptr<State>> states_;           ///< Vector of all the states in the state graph

        ros::NodeHandlePtr node_handle_p;                      ///< Used for initializing states and their
                                                               ///< respective ros pubilshers and subscribers


        bool initialized_ = false;


        /**
         * @brief      Checks if the start state and the end state is connected using a breadth first search.
         *
         * @param[in]  start_state_identifier  The start state identifier
         * @param[in]  end_state_identifier    The end state identifier
         *
         * @return     { description_of_the_return_value }
         */
        bool breadthFirstSearch(std::string start_state_identifier, std::string end_state_identifier);

    public:

        std::shared_ptr<State> current_state_p;                ///< The current state of the state graph

        /**
         * Initializes the graph.
         */
        Graph();

        /**
         * Adds edges to the graph, these edges form the connections between the nodes in the graph. In other words
         * these edges specify which state changes are allowed.
         *
         * @param edges Vector of edges.
         */
        void addEdges(std::vector<fluid::Edge> const &edges);

        /**
         * @return The states in the state graph.
         */
        std::vector<std::shared_ptr<fluid::State>> getStates();

        /**
         * Finds the plan for going from the specified start state to the specified end state (bredth-first search).
         *
         * @param start_state_identifier The identifier of the start state we begin at.
         * @param end_state_identifier The identifier of the end state we want to transition to.
         * @return Vector of states one has to transition to in order to get to the final state.
         */
        std::vector<std::shared_ptr<fluid::State>> getPlanToEndState(std::string start_state_identifier,
                                                                     std::string end_state_identifier);

        /**
         * @return The state with the given identifier.
         */
        std::shared_ptr<fluid::State> getStateWithIdentifier(std::string identifier);

        /**
         * Prints the graph.
         */
        void print();

        /**
         * @brief      Determines if initialized.
         *
         * @return     True if initialized, False otherwise.
         */
        bool isInitialized();

        /**
         * Initializes the state graph with a set of states.
         * 
         * @param refresh_rate The refresh rate the states should be initialized to operate at.
         */
        void initialize(unsigned int refresh_rate);

        /**
         * @return ROS node handle pointer, used when states need to do something specific with ROS.
         */
        ros::NodeHandlePtr getNodeHandlePtr();
    };
}

#endif /* FLUID_FSM_GRAPH_H */
