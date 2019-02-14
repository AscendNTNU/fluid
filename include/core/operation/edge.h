//
//  Created by Simen Gangstad on 09/10/2018.
//

#ifndef FLUID_FSM_EDGE_H
#define FLUID_FSM_EDGE_H

#include "../state.h"
#include <memory>

namespace fluid {
    /** \class Edge
     *  \brief Represents an edge/connection between two nodes a the graph.
     */
    template <class T> class Edge {
    public:
        
        const T source;                 ///< The source of the edge
        const T destination;            ///< The desination of the edge
        
        /** Initializes the edge with its respective source and destination.
         * 
         */
        Edge(T source, T destination): source(source), destination(destination) {}
    };
}
#endif /* FLUID_FSM_EDGE_H */
