#include "core.h"

int Core::refresh_rate = 20;
bool Core::auto_arm = false;
bool Core::auto_set_offboard = false;

std::shared_ptr<StatusPublisher> Core::status_publisher_ptr;

double Core::distance_completion_threshold = 0.3;
double Core::velocity_completion_threshold = 0.1;
double Core::yaw_completion_threshold = 0.2;
double Core::default_height = 1.0;

std::shared_ptr<StatusPublisher> Core::getStatusPublisherPtr() {
    if (!status_publisher_ptr) {
        status_publisher_ptr = std::make_shared<StatusPublisher>();
    }

    return status_publisher_ptr;
}