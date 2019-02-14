#include "../../include/tools/land_detector.h"
#include "../../include/tools/pose_util.h"
#include <cmath>

fluid::LandDetector::LandDetector() : pose_subscriber_(node_handle_.subscribe("mavros/local_position/pose", 
    												   						  1000, 
    																		  &LandDetector::poseCallback, 
    																		  this)),
    								  velocity_subscriber_(node_handle_.subscribe("mavros/local_position/velocity",
    														  	  				  1000, 
    																			  &LandDetector::velocityCallback,
    																			  this)) {}

void fluid::LandDetector::poseCallback(const geometry_msgs::PoseStampedPtr pose_p) {
	current_pose_.pose = pose_p->pose;
	current_pose_.header = pose_p->header;
}

void fluid::LandDetector::velocityCallback(const geometry_msgs::TwistStampedPtr twist_p) {
	current_velocity_.twist = twist_p->twist;
	current_velocity_.header = twist_p->header;
}

bool fluid::LandDetector::hasLanded(mavros_msgs::PositionTarget land_position) {

	return PoseUtil::distanceBetween(current_pose_, land_position) < 0.05 && 
		   std::abs(current_velocity_.twist.linear.z) < 0.05;
}