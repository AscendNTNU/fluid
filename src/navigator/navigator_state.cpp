//
// Created by simengangstad on 08.01.19.
//

#include "../../include/navigator/navigator_state.h"

 fluid::NavigatorState::NavigatorState(fluid::OperationIdentifier identifier, std::string px4_mode) :
 State(std::move(identifier),
 	   px4_mode, 
 	   "state_estimator_pose",
       std::make_shared<fluid::NavigatorPosePublisher>("navigator_pose_topic")) {}