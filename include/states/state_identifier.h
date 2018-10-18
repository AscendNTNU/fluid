//
//  Created by Simen Gangstad on 15/10/2018.
//

#ifndef FLUID_FSM_STATE_IDENTIFIER_H
#define FLUID_FSM_STATE_IDENTIFIER_H

#include <ostream>

namespace fluid {
    enum class StateIdentifier {
        init,
        idle,
        take_off,
        hold,
        move,
        land
    };
}

std::ostream& operator<<(std::ostream& os, const fluid::StateIdentifier identifier);

#endif /* state_identifier_h */

