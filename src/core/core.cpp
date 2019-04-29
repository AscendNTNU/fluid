#include "../../include/core/core.h"


unsigned int fluid::Core::refresh_rate = 30;
unsigned int fluid::Core::message_queue_size = 100;
bool fluid::Core::auto_arm = false;
bool fluid::Core::auto_set_offboard = false;

std::shared_ptr<fluid::StateGraph> fluid::Core::graph_p_;
std::shared_ptr<fluid::StatusPublisher> fluid::Core::status_publisher_p_;

double fluid::Core::minX = 0.0;
double fluid::Core::minY = 0.0;
double fluid::Core::minZ = 0.0;
double fluid::Core::maxX = 0.0;
double fluid::Core::maxY = 0.0;
double fluid::Core::maxZ = 0.0;

double fluid::Core::positionFollowHeight = 2.3;

std::shared_ptr<fluid::StateGraph> fluid::Core::getGraphPtr() {
	if (!graph_p_) {
		graph_p_ = std::make_shared<fluid::StateGraph>();
	}

	return graph_p_;
}

std::shared_ptr<fluid::StatusPublisher> fluid::Core::getStatusPublisherPtr() {
	if (!status_publisher_p_) {
		status_publisher_p_ = std::make_shared<fluid::StatusPublisher>();
	}

	return status_publisher_p_;
}