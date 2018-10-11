//
// Created by simengangstad on 04.10.18.
//

#include "../../include/core/transition.h"
#include <algorithm>

void Transition::perform() {

    TransitionErrorCode transition_error_code = no_error;

    // Copy pose to the new state
    end_state_p->pose = start_state_p->pose;

    // TODO: Communicate with PX4 and get errors (if any) and set them in the error code

    const std::type_info& start_state_type_info = typeid(start_state_p.get());
    const std::type_info& end_state_type_info = typeid(end_state_p.get());

    TransitionError transition_error = {transition_error_code,
                                        start_state_type_info.name(),
                                        end_state_type_info.name()};

    if (auto transition_delegate = transition_delegate_p.lock()) {
        transition_delegate->completed(transition_error);
    }
}
