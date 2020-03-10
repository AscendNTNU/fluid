/**
 * @file init_state.cpp
 */

#include "init_state.h"

#include "core.h"
#include "mavros_interface.h"

bool InitState::hasFinishedExecution() const {
    return true;
}

void InitState::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.establishContactToPX4();
    Core::getStatusPublisherPtr()->status.linked_with_px4 = 1;
    Core::getStatusPublisherPtr()->publish();

    mavros_interface.requestArm(Core::auto_arm);
    Core::getStatusPublisherPtr()->status.armed = 1;
    Core::getStatusPublisherPtr()->publish();

    mavros_interface.requestOffboard(Core::auto_set_offboard);
    Core::getStatusPublisherPtr()->status.px4_mode = "offboard";
    Core::getStatusPublisherPtr()->publish();
}
