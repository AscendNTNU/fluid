#ifndef FLUID_RACING_CONTROLLER_H
#define FLUID_RACING_CONTROLLER_H

#include "controller.h"
namespace fluid {
    class RacingController : Controller {

        public:

        RacingController(const std::string& topic, const unsigned int& degree);

        void tick(std::shared_ptr<std::vector<std::vector<float>>> spline) const override;
    };
}

#endif