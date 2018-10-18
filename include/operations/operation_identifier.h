//
// Created by simengangstad on 18.10.18.
//

#ifndef FLUID_FSM_OPERATION_IDENTIFIER_H
#define FLUID_FSM_OPERATION_IDENTIFIER_H

#include <ostream>

namespace fluid {
    /** \enum OperationIdentifier
     *  \brief Makes it easy to distinguish between different operations.
     *
     */
    enum class OperationIdentifier {
        take_off,
        move,
        land
    };
}

/**
 * Outputs the operation identifier as an output stream.
 *
 * @param os The output stream to output to.
 * @param identifier The identifier to output.
 *
 * @return The output stream with the identifier appended.
 */
std::ostream& operator<<(std::ostream& os, const fluid::OperationIdentifier identifier);


#endif //FLUID_FSM_OPERATION_IDENTIFIER_H
