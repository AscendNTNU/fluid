#include "core.h"

int fluid::Core::refresh_rate = 20;
int fluid::Core::message_queue_size = 10;
bool fluid::Core::auto_arm = false;
bool fluid::Core::auto_set_offboard = false;

std::map<fluid::ControllerType, fluid::ControllerConfig> fluid::Core::controller_config_map {
	{fluid::ControllerType::Racing, fluid::ControllerConfig {1.0, 0.0, 1.5, 20, 15}},
	{fluid::ControllerType::Exploration, fluid::ControllerConfig {1.0, 0.0, 1.5, 1.0, 1.0}}
};

fluid::ControllerType fluid::Core::current_controller_type = fluid::ControllerType::Exploration;

std::shared_ptr<fluid::StateGraph> fluid::Core::graph_ptr_;
std::shared_ptr<fluid::StatusPublisher> fluid::Core::status_publisher_ptr_;
std::shared_ptr<fluid::Controller> fluid::Core::controller_ptr_;

double fluid::Core::distance_completion_threshold = 0.3;
double fluid::Core::velocity_completion_threshold = 0.1;
double fluid::Core::yaw_completion_threshold = 0.2;
double fluid::Core::default_height = 1.0;
double fluid::Core::positionFollowHeight = 2.3;

std::shared_ptr<fluid::StateGraph> fluid::Core::getGraphPtr() {
	if (!graph_ptr_) {
		graph_ptr_ = std::make_shared<fluid::StateGraph>();
	}

	return graph_ptr_;
}

std::shared_ptr<fluid::StatusPublisher> fluid::Core::getStatusPublisherPtr() {
	if (!status_publisher_ptr_) {
		status_publisher_ptr_ = std::make_shared<fluid::StatusPublisher>();
	}

	return status_publisher_ptr_;
}


fluid::ControllerConfig fluid::Core::getControllerConfig() {
	return Core::controller_config_map[Core::current_controller_type];
}

std::shared_ptr<fluid::Controller> fluid::Core::getControllerPtr() {

	if (!controller_ptr_) {
		const fluid::ControllerConfig current_controller_config = Core::controller_config_map[Core::current_controller_type];
		controller_ptr_ = std::make_shared<fluid::Controller>(PID(current_controller_config.kp, 
																  current_controller_config.ki, 
																  current_controller_config.kd));
	}

	return controller_ptr_;
}

void fluid::Core::swapController(const fluid::ControllerType& controller_type) {
	current_controller_type = controller_type;
	const fluid::ControllerConfig current_controller_config = Core::controller_config_map[Core::current_controller_type];

	controller_ptr_ = std::make_shared<fluid::Controller>(fluid::PID(current_controller_config.kp, 
																	 current_controller_config.ki, 
																	 current_controller_config.kd));
}

void fluid::Core::updateControllerConfig(const fluid::ControllerType& controller_type, const fluid::ControllerConfig& controller_config) {
	Core::controller_config_map[controller_type] = controller_config;
}
	