//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_EDGE_H
#define FLUID_FSM_EDGE_H

#include "../state.h"
#include <memory>

namespace fluid {
    /** \class Edge
     *  \brief Represents an edge/connection between two nodes in the graph.
     *
     *  This class encapsulates a transition, which is the binding part between two states
     * (nodes)
     */
    class Edge {
    public:
        
        const std::shared_ptr<State> source;       ///< The source state of the edge
        const std::shared_ptr<State> destination;  ///< The desination state of the edge
        
        
        /** Initializes the Edge with a transition between two states.
         */
        Edge(std::shared_ptr<State> source, std::shared_ptr<State> destination): source(source), destination(destination) {}
    };
}
#endif /* FLUID_FSM_EDGE_H */
