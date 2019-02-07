#include "../../include/core/core.h"


std::unique_ptr<fluid::StateGraph> Core::graph_p_;
std::unique_ptr<fluid::StatusPublisher> Core::status_publisher_p_;


fluid::StateGraph Core::getGraph() {
	if (!graph_p_) {
		graph_p_ = std::make_unique<fluid::StateGraph>();
	}

	return graph_p_.get();
}

fluid::StateGraph Core::getStatusPublisher() {
	if (!status_publisher_p_) {
		status_publisher_p_ = std::make_unique<fluid::StatusPublisher>();
	}

	return status_publisher_p_.get();
}