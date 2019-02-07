//
// Created by simengangstad on 08.01.19.
//

#include "../../include/navigator/navigator_state.h"

 fluid::NavigatorState::NavigatorState(fluid::OperationIdentifier identifier) :
 State(std::move(identifier), 
 	   "state_estimator_pose",
       std::make_shared<fluid::NavigatorPosePublisher>("navigator_pose_topic")) {}