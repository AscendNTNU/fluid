#ifndef FLUID_FSM_CORE_H
#define FLUID_FSM_CORE_H

#include "status_publisher.h"
#include "state_graph.h"
#include "controller.h"

#include <memory>

#include <geometry_msgs/Point32.h>

namespace fluid {

	class Core {

	private:

		// Constructors are private as this class is used as a singleton with members.

		Core() {};
		Core(Core const&) {};
		Core& operator=(Core const&) {};

		static std::shared_ptr<fluid::StateGraph> graph_ptr_;				  ///< Interface for publishing 
																			  ///< status about the fsm.
		
		static std::shared_ptr<fluid::StatusPublisher> status_publisher_ptr_; ///< Provides the states which the
		                                                                 	  ///< operation can consists
        	                                                 				  ///< of and how they are connected.

		static std::shared_ptr<fluid::Controller> controller_ptr_;

	public: 

		static int refresh_rate;											///< The unified refresh rate across 
																			///< the state machine.
																			///< Specifies how fast the operation
																			///< server and its underlying servies 
																			///< will run. E.g. a refresh rate of 10
																			///< hertz will make the states and
																			///< transitions execute their logic every 
																			///< 100 ms (in best case).

		static int message_queue_size;										///< The unified message queue size for 
																			///< the different ros components within
																			///< the FSM.

		static bool auto_arm;		 										///< Determines whether the state machine
																			///< should auto arm or not.

		static bool auto_set_offboard;										///< Determines whether the state machine
																			///< should set itself into offboard
																			///< automatically or not.

		static double distance_completion_threshold;						///< Specifies the radius for within we 
																			///< we can say that we are at the given position. 

		static double velocity_completion_threshold;						///< Specifies how low the velocity has to 
																			///< be before we issue that a given state
																			///< is completed. This serves the purpose to prevent
																			///< oscillations in the case when new operations are
																			///< fired the moment the previous completes.
																			///< With this threshold, we have to wait for the 
																			///< drone to be steady at the current position before
																			///< carrying on.

		static double yaw_completion_threshold;								///< Specifies the threshold for tolerance
																			///< of yaw, so the drone's yaw has to be within 
																			///< this threshold before we continue doing another
																			///< operation.

		static double default_height;										///< The default height to operate at for
																			///< take off and move if none is specified.
		
		static double positionFollowHeight;									///< The height at which the drone will
																			///< follow a given target.	

		static std::shared_ptr<fluid::StateGraph> getGraphPtr();
		static std::shared_ptr<fluid::StatusPublisher> getStatusPublisherPtr(); 
		static void swapController(std::shared_ptr<Controller> controller_ptr);
		static std::shared_ptr<fluid::Controller> getControllerPtr();
	};
}

#endif