#ifndef FLUID_EXPLORATION_STATE_H
#define FLUID_EXPLORATION_STATE_H

#include "move_state.h"
#include "state_identifier.h"

namespace fluid {
    class ExplorationState : public MoveState {

        private:

            geometry_msgs::Point point_of_interest;
            ros::Subscriber point_of_interest_subscriber;

            void pointOfInterestCallback(const geometry_msgs::PointConstPtr& point);
            
        public:
            ExplorationState() : MoveState(fluid::StateIdentifier::Exploration, 1.0, 0.3, 1.0) {}

            void initialize() override;
            void tick() override;
    };
}

#endif