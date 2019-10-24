#ifndef FLUID_ROTATE_STATE_H
#define FLUID_ROTATE_STATE_H

#include "state.h"
#include "util.h"

namespace fluid {

    /** \class RotateState 
     *  \brief Represents the state where the is rotating at the current position.
     */
    class RotateState : public State {
    private:

        geometry_msgs::Point initial_position;

    public:

        explicit RotateState() : State(fluid::StateIdentifier::Rotate, fluid::PX4::Offboard, false, true) {}

        bool hasFinishedExecution() const override;
        void initialize() override;

        std::vector<std::vector<double>> getSplineForPath(const std::vector<geometry_msgs::Point>& path) const override;
        ControllerType getPreferredController() const override;
    };
}

#endif 
