//
//  Created by Simen Gangstad on 15/10/2018.
//

#ifndef FLUID_FSM_STATE_IDENTIFIER_H
#define FLUID_FSM_STATE_IDENTIFIER_H

#include <ostream>

namespace fluid {
    /** \enum StateIdentifier
     *  \brief Makes it easy to distinguish between different states.
     *
     */
    enum class StateIdentifier {
        init,
        idle,
        take_off,
        hold,
        move,
        land
    };
}

/**
 * Outputs the state identifier as an output stream.
 *
 * @param os The output stream to output to.
 * @param identifier The identifier to output.
 *
 * @return The output stream with the identifier appended.
 */
std::ostream& operator<<(std::ostream& os, const fluid::StateIdentifier identifier);

#endif /* state_identifier_h */

