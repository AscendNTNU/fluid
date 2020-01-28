#ifndef FOLLOW_MAST_STATE_H 
#define FOLLOW_MAST_STATE_H 

#include "state.h"
#include "state_identifier.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class FollowMastState : public State {

private:
    geometry_msgs::PoseWithCovarianceStamped module_info;
    ros::Subscriber module_position_subscriber;



public:
    explicit FollowMastState() 
        :   State(StateIdentifier::FollowMast, PX4StateIdentifier::Offboard, false, false),
            module_position_subscriber(node_handle.subscribe("/ai/ue4/module_pos", 10, &FollowMastState::modulePositionCallback, this)) 
    
    {}

    void modulePositionCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr module_position); 
    bool hasFinishedExecution() const override;
    void initialize() override;
    void tick() override;

};


#endif


