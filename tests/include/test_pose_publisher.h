//
// Created by simengangstad on 28.10.18.
//

#ifndef FLUID_FSM_TEST_PUBLISHER_H
#define FLUID_FSM_TEST_PUBLISHER_H

#include "../../include/core/pose_publisher.h"
#include <geometry_msgs/PoseStamped.h>

class TestPosePublisher: public fluid::PosePublisher {
public:

    void publish(geometry_msgs::PoseStamped pose_stamped);
};

#endif //FLUID_FSM_TEST_PUBLISHER_H
