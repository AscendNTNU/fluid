#ifndef FLUID_EXPLORATION_STATE_H
#define FLUID_EXPLORATION_STATE_H

#include "move_state.h"
#include "state_identifier.h"

namespace fluid {
    class ExplorationState : public MoveState {

        private:

            bool retrieved_point_of_interest = false;
            geometry_msgs::Point point_of_interest;
            ros::Subscriber point_of_interest_subscriber;

            void pointOfInterestCallback(const geometry_msgs::PointConstPtr& point);
            
        public:
            ExplorationState() : MoveState(fluid::StateIdentifier::Exploration, 0.3, 0.3, 0.3) {}

            void initialize() override;
            void tick() override;
    };
}

#endif