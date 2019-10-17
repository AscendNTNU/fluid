#ifndef FLUID_RACING_CONTROLLER_H
#define FLUID_RACING_CONTROLLER_H

#include "controller.h"

namespace fluid {
    class RacingController : public Controller {

        public:

        RacingController(const std::string& topic);
    };
}

#endif