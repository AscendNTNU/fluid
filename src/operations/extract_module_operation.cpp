/**
 * @file extract_module_operation.cpp
 */
#include "extract_module_operation.h"

#include "mavros_interface.h"
#include "util.h"

#include <std_srvs/SetBool.h>

ExtractModuleOperation::ExtractModuleOperation() : Operation(OperationIdentifier::EXTRACT_MODULE, false) {
    module_pose_subscriber =
        node_handle.subscribe("/airsim/module_position", 10, &ExtractModuleOperation::modulePoseCallback, this);
    backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");
}

void ExtractModuleOperation::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.setParam("WPNAV_SPEED", speed);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed);

    //mavros_interface.setParam("ANGLE_MAX", 20);
    //mavros_interface.setParam("WPNAV_SPEED_DN", 0.5);

    // Use the current position as setpoint until we get a message with the module position
    setpoint.position = getCurrentPose().pose.position;
}

bool ExtractModuleOperation::hasFinishedExecution() const { return module_state == ModuleState::EXTRACTED; }

void ExtractModuleOperation::modulePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr) {
    module_pose = *module_pose_ptr;
}

void ExtractModuleOperation::tick() {
    setpoint.type_mask = TypeMask::POSITION;

    // Wait until we get the first module position readings before we do anything else.
    if (module_pose.header.seq == 0) {
        return;
    }

    const double dx = module_pose.pose.pose.position.y - getCurrentPose().pose.position.x;
    const double dy = module_pose.pose.pose.position.x - getCurrentPose().pose.position.y;

    setpoint.yaw = std::atan2(dy, dx) - M_PI / 18.0;

    const double distance_to_module = sqrt(dx * dx + dy * dy);

    const double dvx = getCurrentTwist().twist.linear.x;
    const double dvy = getCurrentTwist().twist.linear.y;
    const double dvz = getCurrentTwist().twist.linear.z;

    const double speed = sqrt(dvx * dvx + dvy * dvy + dvz * dvz);

    switch (module_state) {
        case ModuleState::APPROACHING: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            // TODO: This has to be fixed, should be facing towards the module from any given position,
            // not just from the x direction
            setpoint.position.y = module_pose.pose.pose.position.x + 1.5;
            setpoint.position.z = module_pose.pose.pose.position.z;

            if (distance_to_module < 1.8) {
                module_state = ModuleState::OVER;
            }

            break;
        }
        case ModuleState::OVER: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z + 0.3;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.1 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                module_state = ModuleState::BEHIND_WITH_HOOKS;
            }

            break;
        }
        case ModuleState::BEHIND_WITH_HOOKS: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 0.78;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.05 && getCurrentTwist().twist.linear.z < 0.03 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                module_state = ModuleState::EXTRACTING;
            }

            break;
        }
        case ModuleState::EXTRACTING: {
            setpoint.position.x = module_pose.pose.pose.position.y;
            setpoint.position.y = module_pose.pose.pose.position.x + 2.0;
            setpoint.position.z = module_pose.pose.pose.position.z - 0.1;

            if (!called_backpropeller_service) {
                std_srvs::SetBool request;
                request.request.data = true;
                backpropeller_client.call(request);
                called_backpropeller_service = true;
            }           

            // If the module is on the way down
            // TODO: this should be checked in a better way
            if (module_pose.pose.pose.position.z < 0.5) {
                module_state = ModuleState::EXTRACTED;
                std_srvs::SetBool request;
                request.request.data = false;
                backpropeller_client.call(request);
                called_backpropeller_service = false;
            }

            break;
        }
    }
}
