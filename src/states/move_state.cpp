//
// Created by simengangstad on 11.10.18.
//

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>

#include "move_state.h"
#include "util.h"
#include "core.h"

bool fluid::MoveState::hasFinishedExecution()
{
    bool atPositionTarget = Util::distanceBetween(current_pose_.pose.position, setpoint.position) < fluid::Core::distance_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.x) < fluid::Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.y) < fluid::Core::velocity_completion_threshold &&
                            std::abs(getCurrentTwist().twist.linear.z) < fluid::Core::velocity_completion_threshold;

    bool atYawTarget = std::abs(Util::angleBetween(current_pose_.pose.orientation, setpoint.yaw)) < fluid::Core::yaw_completion_threshold;

    return atYawTarget && atPositionTarget;
}

unsigned long factorial(unsigned int n)
{
    unsigned long long factorial = 1;

    for (int i = 1; i <= n; ++i)
    {
        factorial *= i;
    }

    return factorial;
}

void fluid::MoveState::initialize()
{
    if (setpoint.position.z <= 0.1)
    {
        setpoint.position.z = fluid::Core::default_height;
    }
}

void fluid::MoveState::tick()
{
    setpoint.type_mask = fluid::TypeMask::Default;

    std::vector<float> x = {100.0f / factorial(13), 0, -100.0f / factorial(11), 0, 100.0f / factorial(9), 0, -100.0f / factorial(7), 0, 100.0f / factorial(5), 0.0, -100.0f / factorial(3), 0.0, 100, 0};
    std::vector<float> y = {10.0f / factorial(12), 0, -10.0f / factorial(10), 0, 10.0f / factorial(8), 0, -10.0f / factorial(6), 0, 10.0f / factorial(4), 0.0, -10.0f / factorial(2), 0, 10};
    std::vector<float> z = {0, 0, 0, 0, 0};
    std::vector<std::vector<float>> test_vec;
    test_vec.push_back(x);
    test_vec.push_back(y);
    test_vec.push_back(z);

    ros::Duration duration = ros::Time::now() - startTime;

    std::shared_ptr<std::vector<std::vector<float>>> spline = std::make_shared<std::vector<std::vector<float>>>(test_vec);

    Core::getControllerPtr()->tick(duration.toSec(), spline);
}