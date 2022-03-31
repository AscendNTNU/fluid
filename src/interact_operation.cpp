/**
 * @file interact_operation.cpp
 */
#include "fluid/interact_operation.hpp"

#include "fluid/util.hpp"
#include "fluid/type_mask.hpp"

#include <functional>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
//#include <ascend_msgs/SetInt.hpp>
#include <rclcpp/time.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/header.hpp>
#include <chrono>
//A list of parameters for the user
#define MAST_INTERACT false //safety feature to avoid going at close proximity to the mast and set the FH
#define MAX_DIST_FOR_CLOSE_TRACKING     1.0 //max distance from the mast before activating close tracking
#define TIME_WINDOW_INTERACTION         1.0 //window within the drone is allowed to go to the OVER state    

  // Important distances                  // best_sim // theoretical_sim
//#define DIST_FH_DRONE_CENTRE_X   0.42   // 0.42     // 0.5377
//#define DIST_FH_DRONE_CENTRE_Y   0.02   // 0.02     // 0.5377
//#define DIST_FH_DRONE_CENTRE_Z  -0.2914 //-0.4914   //-0.3214

#define SAVE_DATA   true
#define SAVE_Z      false

#define TIME_TO_COMPLETION 0.5 //time in sec during which we want the drone to succeed a state before moving to the other.
#define APPROACH_ACCURACY 0.1 //Accuracy needed by the drone to go to the next state

#define MAX_ANGLE   1500 // in centi-degrees 

using std::placeholders::_1;

uint16_t time_cout = 0; //used not to do some stuffs at every tick


geometry_msgs::msg::PoseStamped prev_module_pose;
geometry_msgs::msg::Vector3 DIST_FH_DRONE_CENTRE;


//function called when creating the operation
InteractOperation::InteractOperation(const float& fixed_mast_yaw,
    const bool& autoPublish, MastNodeConfiguration config, const float& offset) 
        : Node("mast_node"), config(config), autoPublish(autoPublish) { 

    mast = Mast(this, fixed_mast_yaw);

    pose_subscriber = 
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("mavros/global_position/local", 1, std::bind(&InteractOperation::poseCallback, this, _1));
        //Change name of topic
    twist_subscriber =
        this->create_subscription<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local", 1, std::bind(&InteractOperation::twistCallback, this, _1));
        //Change name of topic

    setpoint_publisher = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    //setpoint_publisher = this->create_publisher<>("", 10)
    //Change message type, maybe convert to function so its callable.
    
    setpoint.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;

    rate_int = (int) config.refresh_rate;
    
    SHOW_PRINTS = config.interaction_show_prints;
    EKF = config.ekf;
    USE_PERCEPTION = config.use_perception;
    MAX_ACCEL = config.interact_max_acc;
    MAX_VEL = config.interact_max_vel;
    DIST_FH_DRONE_CENTRE.x = config.fh_offset[0];
    DIST_FH_DRONE_CENTRE.y = config.fh_offset[1];
    DIST_FH_DRONE_CENTRE.z = config.fh_offset[2];

    //Choose an initial offset. It is the offset for the approaching state.
    //the offset is set in the frame of the mast:    
    desired_offset.x = offset;     //forward
    desired_offset.y = DIST_FH_DRONE_CENTRE.y;     //left
    desired_offset.z = DIST_FH_DRONE_CENTRE.z+0.03;    //up
}

void InteractOperation::initialize() {

    if(EKF){
        ekf_module_pose_subscriber = this->create_subscription<mavros_msgs::msg::PositionTarget>("/ekf/module/state",
                                     10, std::bind(&InteractOperation::ekfModulePoseCallback, this, _1));
        ekf_state_vector_subscriber = this->create_subscription<mavros_msgs::msg::DebugValue>("/ekf/state",
                                     10, std::bind(&InteractOperation::ekfStateVectorCallback, this, _1));
        gt_module_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr>("/simulator/module/ground_truth/pose",
                                     10, std::bind(&InteractOperation::gt_modulePoseCallbackWithCov, this, _1));
        RCLCPP_INFO(rclcpp::get_logger("interact_operation"), "/fluid: Uses EKF data");
    }
    else{
    module_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/simulator/module/ground_truth/pose",
                                    10, std::bind(&InteractOperation::gt_modulePoseCallbackWithCov, this, _1));
    }
    fh_state_subscriber = this->create_subscription<std_msgs::msg::Bool>("/fh_interface/fh_state",
                                    10, std::bind(&InteractOperation::FaceHuggerCallback, this, _1));
    close_tracking_ready_subscriber = this->create_subscription<std_msgs::msg::Bool>("/close_tracking_running",
                                    10, std::bind(&InteractOperation::closeTrackingCallback, this, _1));

    // start_close_tracking_client = this->create_client<ascend_msgs::msg::SetInt>("start_close_tracking");
    pause_close_tracking_client = this->create_client<std_srvs::srv::Trigger>("Pause_close_tracking");

    interact_fail_pub = this->create_publisher<std_msgs::msg::Int16>("/fluid/interact_fail",10);
    altitude_and_yaw_pub = this->create_publisher<mavros_msgs::msg::PositionTarget>("/mavros/setpoint_raw/local",10);
    
    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;
    setpoint.header.frame_id = "map";

    transition_state.state.position = desired_offset;
    transition_state.cte_acc = MAX_ACCEL; 
    transition_state.max_vel = MAX_VEL;
    
    faceHugger_is_set = false;    
    close_tracking_is_set = false;
    close_tracking_is_ready = false;

    #if SAVE_DATA
    reference_state = DataFile("reference_state.txt");
    drone_pose = DataFile("drone_pose.txt");
    gt_reference = DataFile("gt_reference.txt");

    reference_state.shouldSaveZ(SAVE_Z);
    drone_pose.shouldSaveZ(SAVE_Z);
    gt_reference.shouldSaveZ(SAVE_Z);

    reference_state.initStateLog();
    drone_pose.initStateLog();    
    #if SAVE_Z
    gt_reference.init("Time\tpose.x\tpose.y\tpose.z");
    #else
    gt_reference.init("Time\tpose.x\tpose.y");
    #endif
    #endif
    approaching_t0 = this->now();
    completion_count =0;
}

bool InteractOperation::hasFinishedExecution() const {
    return interaction_state == InteractionState::EXTRACTED; 
}

void InteractOperation::ekfModulePoseCallback(
                const mavros_msgs::msg::PositionTarget module_state) {
    mast.updateFromEkf(module_state);
    //Tells code that we are ready to try arate_int
    }

void InteractOperation::gt_modulePoseCallbackWithCov(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr module_pose_ptr) {
    geometry_msgs::msg::PoseStamped module_pose;
    module_pose.header = module_pose_ptr->header;
    module_pose.pose = module_pose_ptr->pose.pose;
    gt_modulePoseCallback(module_pose);
}

void InteractOperation::gt_modulePoseCallbackWithoutCov(geometry_msgs::msg::PoseStamped::SharedPtr module_pose_ptr){
    gt_modulePoseCallback(*module_pose_ptr);
}

void InteractOperation::gt_modulePoseCallback(
    const geometry_msgs::msg::PoseStamped module_pose) {
    float module_pose_time = module_pose.header.stamp.sec*1.0 + module_pose.header.stamp.nanosec*1e-9;
    float prev_module_pose_time = prev_module_pose.header.stamp.sec*1.0 + prev_module_pose.header.stamp.nanosec*1e-9;
    //Tells code that we are ready to try and run. 
    rcvd_module_pose = true;

    if((module_pose_time - prev_module_pose_time) > 0.01){
        #if SAVE_DATA
            prev_module_pose = module_pose;
            mavros_msgs::msg::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
            geometry_msgs::msg::Vector3 vec;
            vec.x = module_pose.pose.position.x + smooth_rotated_offset.position.x;
            vec.y = module_pose.pose.position.y + smooth_rotated_offset.position.y;
            vec.z = module_pose.pose.position.z + smooth_rotated_offset.position.z;
            gt_reference.saveVector3(vec);
        #endif
        if(!EKF){
            const geometry_msgs::msg::Vector3 received_eul_angle = Util::quaternion_to_euler_angle(module_pose.pose.orientation);
            mast.update(module_pose);
            mast.search_period(received_eul_angle.x); //pitch is y euler angle because of different frame
        }
    }
}

void InteractOperation::FaceHuggerCallback(const std_msgs::msg::Bool released){
    if (released.data && !faceHugger_is_set){
        RCLCPP_INFO(rclcpp::get_logger("interact_operation"), "CONGRATULATION, FaceHugger set on the mast! We can now exit the mast");
        interaction_state =  InteractionState::EXIT;
        faceHugger_is_set = true;

        desired_offset.x = 2.0;   //forward
        desired_offset.y = DIST_FH_DRONE_CENTRE.y;    //left
        desired_offset.z = DIST_FH_DRONE_CENTRE.z - 0.3;   //up
        transition_state.state.position.z = desired_offset.z;
        transition_state.cte_acc = MAX_ACCEL*3;
        transition_state.max_vel = MAX_VEL*3;
        transition_state.finished_bitmask = 0x0;
        //transition_state.state.velocity.x = MAX_VEL*3;
    }
}

void InteractOperation::closeTrackingCallback(std_msgs::msg::Bool ready){
    close_tracking_is_ready = ready.data; 
}


template<typename T>  T rotate2 (T pt, float yaw) {
    T rotated_point;
    rotated_point.x = cos(yaw) * pt.x - sin(yaw) * pt.y;
    rotated_point.y = cos(yaw) * pt.y + sin(yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

mavros_msgs::msg::PositionTarget InteractOperation::rotate(mavros_msgs::msg::PositionTarget setpoint, float yaw){
    mavros_msgs::msg::PositionTarget rotated_setpoint;
    rotated_setpoint.position = rotate2<geometry_msgs::msg::Point>(setpoint.position, yaw);
    rotated_setpoint.velocity = rotate2<geometry_msgs::msg::Vector3>(setpoint.velocity, yaw);
    rotated_setpoint.acceleration_or_force = rotate2<geometry_msgs::msg::Vector3>(setpoint.acceleration_or_force, yaw);

    return rotated_setpoint;
}


void InteractOperation::update_transition_state()
{// try to make a smooth transition when the relative targeted position between the drone
// and the mast is changed
//mavros_msgs::msg::PositionTargetConstPtr ts = &transition_state.state;
// Analysis on the x axis
    if (abs(desired_offset.x - transition_state.state.position.x) >= 0.001){
    // if we are in a transition state on the x axis
        transition_state.finished_bitmask &= ~0x1;
        if (Util::sq(transition_state.state.velocity.x) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.x - transition_state.state.position.x)){
        // if it is time to brake to avoid overshoot
            //set the transition acceleration (or deceleration) to the one that will lead us to the exact point we want
            transition_state.state.acceleration_or_force.x = - Util::sq(transition_state.state.velocity.x) 
                                            /2.0 / (desired_offset.x - transition_state.state.position.x);
        }
        else if (abs(transition_state.state.velocity.x) > transition_state.max_vel){
            //we stop accelerating and maintain speed
            transition_state.state.acceleration_or_force.x = 0.0;
        }
        else{
        //we are in the acceleration phase of the transition){
            if (desired_offset.x - transition_state.state.position.x > 0.0)
                transition_state.state.acceleration_or_force.x = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.x = - transition_state.cte_acc;
        }
        // Whatever the state we are in, update velocity and position of the target
        transition_state.state.velocity.x = transition_state.state.velocity.x + transition_state.state.acceleration_or_force.x / (float)rate_int;
        transition_state.state.position.x = transition_state.state.position.x + transition_state.state.velocity.x / (float)rate_int;
        
    }
    else if (abs(transition_state.state.velocity.x) < 0.1){
        //setpoint reached destination on this axis
        transition_state.state.position.x = desired_offset.x;
        transition_state.state.velocity.x = 0.0;
        transition_state.state.acceleration_or_force.x = 0.0;
        transition_state.finished_bitmask |= 0x1;
    }

// Analysis on the y axis, same as on the x axis
    if (abs(desired_offset.y - transition_state.state.position.y) >= 0.001){
        transition_state.finished_bitmask &= ~0x2;
        if (Util::sq(transition_state.state.velocity.y) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.y - transition_state.state.position.y))
            transition_state.state.acceleration_or_force.y = - Util::sq(transition_state.state.velocity.y) 
                                            /2.0 / (desired_offset.y - transition_state.state.position.y);
        else if (abs(transition_state.state.velocity.y) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.y = 0.0;
        else{
            if (desired_offset.y - transition_state.state.position.y > 0.0)
                transition_state.state.acceleration_or_force.y = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.y = - transition_state.cte_acc;
            }
        transition_state.state.velocity.y  =   transition_state.state.velocity.y  + transition_state.state.acceleration_or_force.y / (float)rate_int;
        transition_state.state.position.y =  transition_state.state.position.y  + transition_state.state.velocity.y / (float)rate_int;
    }
    else if (abs(transition_state.state.velocity.y) < 0.1){
        transition_state.state.position.y = desired_offset.y;
        transition_state.state.velocity.y = 0.0;
        transition_state.state.acceleration_or_force.y = 0.0;
        transition_state.finished_bitmask |= 0x2;
    }

// Analysis on the z axis, same as on the x axis
    if (abs(desired_offset.z - transition_state.state.position.z) >= 0.001){
        transition_state.finished_bitmask &= ~0x4;
        if (Util::sq(transition_state.state.velocity.z) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.z - transition_state.state.position.z))
            transition_state.state.acceleration_or_force.z = - Util::sq(transition_state.state.velocity.z) 
                                            /2.0 / (desired_offset.z - transition_state.state.position.z);
        else if (abs(transition_state.state.velocity.z) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.z = 0.0;
        else {
            if (desired_offset.z - transition_state.state.position.z > 0.0)
                transition_state.state.acceleration_or_force.z = transition_state.cte_acc;
            else 
                transition_state.state.acceleration_or_force.z = - transition_state.cte_acc;
        }
        transition_state.state.velocity.z =  transition_state.state.velocity.z + transition_state.state.acceleration_or_force.z / (float)rate_int;
        transition_state.state.position.z =  transition_state.state.position.z + transition_state.state.velocity.z / (float)rate_int;
    }
    else if (abs(transition_state.state.velocity.z) < 0.1){
        transition_state.state.position.z = desired_offset.z;
        transition_state.state.velocity.z = 0.0;
        transition_state.state.acceleration_or_force.z = 0.0;
        transition_state.finished_bitmask |= 0x4;
    }
}


float InteractOperation::estimate_time_to_mast()
{
    // Estimation of the time it takes to go from current position to interaction point
    float dist = transition_state.state.position.x - DIST_FH_DRONE_CENTRE.x; //assuming that the drone is always accurate
    float dist_acc_decc = Util::sq(MAX_VEL)/MAX_ACCEL;
    if (dist < dist_acc_decc)
        return 2.0 * sqrt(2.0*dist/MAX_ACCEL);
    else
        return (dist - dist_acc_decc) / MAX_VEL //time during max vel
                    + 2 * MAX_VEL / MAX_ACCEL; //time during acceleration and decceleration
}

void InteractOperation::tick() {
    time_cout++;
    mavros_msgs::msg::PositionTarget interact_pt_state = mast.get_interaction_point_state();
    //printf("mast pitch %f, roll %f, angle %f\n", mast_angle.x, mast_angle.y, mast_angle.z);
    // Wait until we get the first module position readings before we do anything else.
    if (!this->rcvd_module_pose) {
        if(time_cout%rate_int==0)
            RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Waiting for interaction point pose callback\n");
        approaching_t0 = this->now();
        return;
    }

    update_transition_state();

    geometry_msgs::msg::Point rotated_offset = rotate2<geometry_msgs::msg::Point>(desired_offset,mast.get_yaw());
    const double dx = interact_pt_state.position.x + rotated_offset.x - getCurrentPose().pose.position.x;
    const double dy = interact_pt_state.position.y + rotated_offset.y - getCurrentPose().pose.position.y;
    const double dz = interact_pt_state.position.z + rotated_offset.z - getCurrentPose().pose.position.z;
    const double distance_to_offset = sqrt(Util::sq(dx) + Util::sq(dy) + Util::sq(dz));
    
    switch (interaction_state) {
        case InteractionState::APPROACHING: {
            if (SHOW_PRINTS and time_cout%rate_int==0) {
                printf("APPROACHING\t");
                printf("distance to ref %f\n", distance_to_offset);
            }
                       
            if(MAST_INTERACT) {
                float time_out_gain = 1 + (this->now() - approaching_t0).seconds()/30.0;
                if ( distance_to_offset <= APPROACH_ACCURACY *time_out_gain ) { 
                    //Todo, we may want to judge the velocity in stead of having a time to completion
                    if (completion_count < ceil(TIME_TO_COMPLETION * (float)rate_int) )
                        completion_count++;
                    else {
                        //We consider that if the drone is ready at some point, it will 
                        //remain ready until it is time to try
                        RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Approaching -> Ready");

                        completion_count = 0;
                        RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Control ready to set the FaceHugger. Waiting for the best opportunity");
                        interaction_state = InteractionState::READY;   
                        desired_offset.x = MAX_DIST_FOR_CLOSE_TRACKING;             
                    }
                }
                else
                    completion_count = 0;
            }
            break;
        }
        case InteractionState::READY: {
            //The drone is ready, we just have to wait for the best moment to go!
            if(!close_tracking_is_set and (transition_state.finished_bitmask & 0x7) == 0x7){
                RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Turning on close tracking");
                
                // send a message to perception to switch close tracking on.
                if(USE_PERCEPTION){
                    // TODO: COMMENT BACK IN!
                    // ascend_msgs::msg::SetInt srv;
                    // srv.request.data = 10;
                    // if (start_close_tracking_client.call(srv)){
                        // close_tracking_is_set = true; 
                    // }
                }
                else{
                    close_tracking_is_set= true; //Todo, to be removed
                    close_tracking_is_ready = true;
                }
            }
            if(mast.time_to_max_pitch() !=-1){ //we don't know it yet
                float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast();

                if(time_to_wait < -TIME_WINDOW_INTERACTION){
                    time_to_wait+=mast.get_period();
                }
                if (SHOW_PRINTS and time_cout%(rate_int/2)==0) {
                    RCLCPP_INFO(rclcpp::get_logger("interact_operation"),"READY; Estimated waiting time before go: ", time_to_wait);
                }
                if( close_tracking_is_ready and (abs(time_to_wait) <= TIME_WINDOW_INTERACTION) )
                { //We are in the good window to set the faceHugger
                    interaction_state = InteractionState::OVER;
                    RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Ready -> Over");
                    desired_offset.x = DIST_FH_DRONE_CENTRE.x;  //forward
                    desired_offset.y = DIST_FH_DRONE_CENTRE.y;   //left
                    desired_offset.z = DIST_FH_DRONE_CENTRE.z+0.03; //up
                    transition_state.cte_acc = MAX_ACCEL;
                    transition_state.max_vel = MAX_VEL;
                    transition_state.finished_bitmask = 0x0;
                }
            }
            break;
        }
        case InteractionState::OVER: {
            if (SHOW_PRINTS and time_cout%(rate_int*2)==0)
                printf("OVER\n");
    
            //We assume that the accuracy is fine, we don't want to take the risk to stay too long
            if ((transition_state.finished_bitmask & 0x7) == 0x7) {
                interaction_state = InteractionState::INTERACT;
                RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Over -> Interact");
                desired_offset.x = DIST_FH_DRONE_CENTRE.x;  //forward
                desired_offset.y = 0.0;   //left
                desired_offset.z -= 0.2;  //up
                transition_state.finished_bitmask = 0x0;
            }
            break;
        }
        case InteractionState::INTERACT: {
            if (SHOW_PRINTS and time_cout%(rate_int*2)==0) 
                printf("INTERACT\n");

            // we don't want to take the risk to stay too long, 
            // Whether the faceHugger is set or not, we have to exit.
            // NB, when FH is set, an interupt function switches the state to EXIT
            if ((transition_state.finished_bitmask & 0x7) == 0x7) {
                interaction_state = InteractionState::EXIT;
                RCLCPP_INFO(rclcpp::get_logger("interact_operation"), "Interact -> Exiting\n",
                "Exit for safety reasons, the FaceHugger could not be placed..."); 

                //we move backward to ensure there will be no colision
                // We directly set the transition state as we want to move as fast as possible
                // and we don't mind anymore about the relative position to the mast
                desired_offset.x = 2;   //forward
                desired_offset.y = DIST_FH_DRONE_CENTRE.y;    //left
                desired_offset.z = DIST_FH_DRONE_CENTRE.z;   //up
                transition_state.state.position = desired_offset;
                transition_state.cte_acc = MAX_ACCEL*3;
                transition_state.max_vel = MAX_VEL*3;
                transition_state.finished_bitmask = 0x0;
                //transition_state.state.velocity.x = MAX_VEL*3;
            }
            break;
        }
        case InteractionState::EXIT: {
            // NB, when FH is set, an interupt function switches the state to EXIT
            if (SHOW_PRINTS and time_cout%(rate_int*2)==0) 
                printf("EXIT\n");
    
            if(close_tracking_is_set){
                if(USE_PERCEPTION){ //Drone is leaving the mast and perception should turn off close tracking
                    
                    //Call service to perception
                    //TODO;

                    // std_srvs::srv::Trigger::Request req;
                    // if (pause_close_tracking_client->async_send_request(req)){
                    //     close_tracking_is_set = false;
                    // }
                }
                else{
                        close_tracking_is_set = false;
                        close_tracking_is_ready = false;
                }
                RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": switch close tracking off");
            }
            
            // Come back the the base or try again.
            if ( distance_to_offset < 0.2 ) {
                if (faceHugger_is_set){
                    RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Exit -> Extracted");
                    interaction_state = InteractionState::EXTRACTED;
                    desired_offset.x = 4;
                    desired_offset.y = DIST_FH_DRONE_CENTRE.y;
                    desired_offset.z = 3;
                    transition_state.cte_acc = MAX_ACCEL*3;
                    transition_state.max_vel = MAX_VEL*3;
                }
                else {
                    RCLCPP_INFO(rclcpp::get_logger("interact_operation"), ": Exit -> Approaching");
                    //ascend_msgs::msg::SetInt interact_fail_srv;
                    number_fail.data++;
                    //interact_fail_srv.request.data = number_fail;
                    for(int i = 0; i<3 ; i ++) interact_fail_pub->publish(number_fail);
                    interaction_state = InteractionState::APPROACHING;
                    desired_offset.x = 2;
                    desired_offset.y = DIST_FH_DRONE_CENTRE.y;
                    desired_offset.z = DIST_FH_DRONE_CENTRE.z+0.03;
                    transition_state.cte_acc = MAX_ACCEL;
                    transition_state.max_vel = MAX_VEL;
                }
            }
            break;
        }
        case InteractionState::EXTRACTED: {
            if (SHOW_PRINTS and time_cout%(rate_int*2)==0) 
                printf("EXTRACTED\n");
            // Operation finished, waiting for AI to close the operation
        }
    }//end switch state

    if (SHOW_PRINTS and time_cout% rate_int ==0) {
        printf("transition pose\tx %f,\ty %f,\tz %f\n",transition_state.state.position.x,
                        transition_state.state.position.y, transition_state.state.position.z);
        geometry_msgs::msg::Point cur_drone_pose = getCurrentPose().pose.position;
        printf("Drone pose\tx %f,\ty %f,\tz %f\tyaw %f\n",cur_drone_pose.x,
                                        cur_drone_pose.y, cur_drone_pose.z,getCurrentYaw());
    }
    
    mavros_msgs::msg::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
    mavros_msgs::msg::PositionTarget ref = Util::addPositionTarget(interact_pt_state,smooth_rotated_offset);

    setpoint.header.stamp = this->now();
    setpoint.yaw = mast.get_yaw()+M_PI;
    setpoint.position = ref.position;
    setpoint.velocity = ref.velocity;

    altitude_and_yaw_pub->publish(setpoint);
    
    #if SAVE_DATA
        reference_state.saveStateLog(ref);
        geometry_msgs::msg::Vector3 drone_acc = rotate2<geometry_msgs::msg::Vector3>(getCurrentAccel(),getCurrentYaw());
        drone_pose.saveStateLog( getCurrentPose().pose.position,getCurrentTwist().twist.linear,drone_acc);
    #endif
}

// void InteractOperation::perform(std::function<bool(void)> should_tick) {

//     // rclcpp::Rate rate(rate_int);
//     // initialize();

//     // do {
//     //     tick();
//     //     if (autoPublish)
//     //         publishSetpoint();
//     //     //rclcpp::spin(this);
//     //     rate.sleep();
//     // } while (rclcpp::ok() && ((should_halt_if_steady && steady) || !hasFinishedExecution()) && should_tick());
//     return;
// }

geometry_msgs::msg::Vector3 InteractOperation::orientation_to_acceleration(geometry_msgs::msg::Quaternion orientation)
{
    geometry_msgs::msg::Vector3 accel;
    geometry_msgs::msg::Vector3 angle = Util::quaternion_to_euler_angle(orientation);
    accel.x = tan(angle.y) *9.81;
    accel.y = -tan(angle.x) *9.81;
    accel.z = 0.0; //we actually don't know ...
    return accel;
}

void InteractOperation::publishSetpoint() { 
    setpoint.header.stamp = this->now();
    setpoint.header.frame_id = "map";
    setpoint_publisher->publish(setpoint); // Fix the formatting of this (i. e. make it command) 
}

void InteractOperation::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
    current_pose.pose = pose->pose.pose;
    current_pose.header = pose->header;
    current_accel = orientation_to_acceleration(pose->pose.pose.orientation);
}

void InteractOperation::twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr twist) {
    current_twist.twist = twist->twist;
    current_twist.header = twist->header;
}

float InteractOperation::getCurrentYaw() const {
    geometry_msgs::msg::Quaternion quaternion = current_pose.pose.orientation;
    geometry_msgs::msg::Vector3 euler = Util::quaternion_to_euler_angle(quaternion);
    return euler.z;
}

geometry_msgs::msg::PoseStamped InteractOperation::getCurrentPose() const { return current_pose; }

geometry_msgs::msg::TwistStamped InteractOperation::getCurrentTwist() const { return current_twist; }

geometry_msgs::msg::Vector3 InteractOperation::getCurrentAccel() const { return current_accel; }