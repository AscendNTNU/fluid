//
// Created by simengangstad on 28.10.18.
//

#include "../include/test_pose_publisher.h"
#include <iostream>

void TestPosePublisher::publish(geometry_msgs::PoseStamped pose_stamped) {
    std::cout << pose_stamped << std::endl;
}