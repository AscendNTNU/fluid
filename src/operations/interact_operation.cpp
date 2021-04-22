/**
 * @file interact_operation.cpp
 */
#include "interact_operation.h"

#include "mavros_interface.h"
#include "util.h"
#include "fluid.h" //to get access to the tick rate
#include "type_mask.h"

#include <std_srvs/SetBool.h>

//A list of parameters for the user
#define MAST_INTERACT false //safety feature to avoid going at close proximity to the mast and set the FH
#define MAX_DIST_FOR_CLOSE_TRACKING     1.0 //max distance from the mast before activating close tracking
    

// Important distances
#define DIST_FH_DRONE_CENTRE    0.28


#define SAVE_DATA   true
#define SAVE_Z      false
#define USE_SQRT    false
#define ATTITUDE_CONTROL 4   //4 = ignore yaw rate   //Attitude control does not work without thrust

#define TIME_TO_COMPLETION 0.5 //time in second during which we want the drone to succeed a state before moving to the other.
#define APPROACH_ACCURACY 0.06 //Accuracy needed by the drone to go to the next state

// Feedforward tuning
#define ACCEL_FEEDFORWARD_X 0.0
#define ACCEL_FEEDFORWARD_Y 0.0

#define MAX_ANGLE   400 // in centi-degrees


uint16_t time_cout = 0; //used not to do some stuffs at every tick

//function called when creating the operation
InteractOperation::InteractOperation(const float& fixed_mast_yaw, const float& offset) : 
            Operation(OperationIdentifier::INTERACT, false, false) { 
    mast = Mast(fixed_mast_yaw);
    
    //get parameters from the launch file.
    const float* temp = Fluid::getInstance().configuration.LQR_gains;
    for (uint8_t i=0 ; i<2 ; i++) { 
        K_LQR_X[i] = temp[2*i];
        K_LQR_Y[i] = temp[2*i+1];
    }
    SHOW_PRINTS = Fluid::getInstance().configuration.interaction_show_prints;
    EKF = Fluid::getInstance().configuration.ekf;
    MAX_ACCEL = Fluid::getInstance().configuration.interact_max_acc;
    MAX_VEL = Fluid::getInstance().configuration.interact_max_vel;

    //Choose an initial offset. It is the offset for the approaching state.
    //the offset is set in the frame of the mast:    
    desired_offset.x = offset;     //forward
    desired_offset.y = 0.0;     //left
    desired_offset.z = -0.45;    //up

    }

void InteractOperation::initialize() {   

    if(EKF){
        ekf_module_pose_subscriber = node_handle.subscribe("/ekf/module/state",
                                     10, &InteractOperation::ekfModulePoseCallback, this);
        ekf_state_vector_subscriber = node_handle.subscribe("/ekf/state",
                                     10, &InteractOperation::ekfStateVectorCallback, this);
        gt_module_pose_subscriber = node_handle.subscribe("/simulator/module/ground_truth/pose",
                                     10, &InteractOperation::modulePoseCallback, this);
    }
    else{
    module_pose_subscriber = node_handle.subscribe("/simulator/module/ground_truth/pose",
                                    10, &InteractOperation::modulePoseCallback, this);
    }
    attitude_pub = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    //creating own publisher to choose exactly when we send messages
    altitude_and_yaw_pub = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    attitude_setpoint.type_mask = ATTITUDE_CONTROL;   
    
    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;

    MavrosInterface mavros_interface;
    mavros_interface.setParam("ANGLE_MAX", MAX_ANGLE);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << MAX_ANGLE/100.0 << " deg.");

    // The transition state is mesured in the mast frame
    transition_state.state.position = desired_offset;
    transition_state.cte_acc = MAX_ACCEL; 
    transition_state.max_vel = MAX_VEL;
    
    faceHugger_is_set = false;    
    close_tracking = false;

    #if SAVE_DATA
    reference_state = DataFile("reference_state.txt");
    drone_pose = DataFile("drone_pose.txt");
    gt_reference = DataFile("gt_reference.txt");

    reference_state.initStateLog();
    drone_pose.initStateLog();    
    gt_reference.init("Time\tpose.x\tpose.y\tpose.z");
    //create a header for the datafiles.
    #endif

    startApproaching = ros::Time::now();
    completion_count =0;
}

bool InteractOperation::hasFinishedExecution() const {
    return interaction_state == InteractionState::EXTRACTED; 
}

void InteractOperation::ekfModulePoseCallback(
                const mavros_msgs::PositionTarget module_state) {
    mast.updateFromEkf(module_state);
}

void InteractOperation::ekfStateVectorCallback(
                const mavros_msgs::DebugValue ekf_state) {
    mast.search_period(ekf_state.data[0]); //the first element is the pitch
}

void InteractOperation::modulePoseCallback(
    const geometry_msgs::PoseStampedConstPtr module_pose_ptr) {
    #if SAVE_DATA
        geometry_msgs::Vector3 vec;
        vec.x = module_pose_ptr->pose.position.x;
        vec.y = module_pose_ptr->pose.position.y;
        vec.z = module_pose_ptr->pose.position.z;
        gt_reference.saveVector3(vec);
    #endif
    if(!EKF){
        ROS_INFO_STREAM("NOT EKF");
        const geometry_msgs::Vector3 received_eul_angle = Util::quaternion_to_euler_angle(module_pose_ptr->pose.orientation);
        mast.update(module_pose_ptr);
        mast.search_period(received_eul_angle.y); //pitch is y euler angle because of different frame
    }
}

void InteractOperation::FaceHuggerCallback(const bool released){
    if (released){
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "CONGRATULATION, FaceHugger set on the mast! We can now exit the mast");
        interaction_state =  InteractionState::EXIT;
        faceHugger_is_set = true;
    }
    else 
        faceHugger_is_set = false;
}


/*template<typename T>  T& rotate2 (T& pt, float yaw) {
    T& rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}
*/
mavros_msgs::PositionTarget InteractOperation::rotate(mavros_msgs::PositionTarget setpoint, float yaw){
    mavros_msgs::PositionTarget rotated_setpoint;
    rotated_setpoint.position = rotate(setpoint.position);
    rotated_setpoint.velocity = rotate(setpoint.velocity);
    rotated_setpoint.acceleration_or_force = rotate(setpoint.acceleration_or_force);

    return rotated_setpoint;
}

geometry_msgs::Vector3 InteractOperation::rotate(geometry_msgs::Vector3 pt, float yaw){
    geometry_msgs::Vector3 rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Point InteractOperation::rotate(geometry_msgs::Point pt, float yaw){
    geometry_msgs::Point rotated_point;
    rotated_point.x = cos(mast.get_yaw()) * pt.x - sin(mast.get_yaw()) * pt.y;
    rotated_point.y = cos(mast.get_yaw()) * pt.y + sin(mast.get_yaw()) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Vector3 InteractOperation::LQR_to_acceleration(mavros_msgs::PositionTarget ref){
    accel_target.z = 0;
    #if USE_SQRT
        accel_target.x = K_LQR_X[0] * Util::signed_sqrt(ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * Util::signed_sqrt(ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_Y[0] * Util::signed_sqrt(ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_Y[1] * Util::signed_sqrt(ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;

    #else
        accel_target.x = K_LQR_X[0] * (ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * (ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_Y[0] * (ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_Y[1] * (ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;
    #endif
    // the right of the mast is the left of the drone: the drone is facing the mast
    accel_target.x = - accel_target.x;
    return accel_target;
}

geometry_msgs::Quaternion InteractOperation::accel_to_orientation(geometry_msgs::Vector3 accel){
    double yaw = mast.get_yaw() + M_PI; //we want to face the mast
    double roll = atan2(accel.y,9.81);
    double pitch = atan2(accel.x,9.81);
    return Util::euler_to_quaternion(yaw, roll, pitch);
}

void InteractOperation::update_attitude_input(mavros_msgs::PositionTarget offset){
    mavros_msgs::PositionTarget ref = Util::addPositionTarget(mast.get_interaction_point_state(), offset);

    attitude_setpoint.header.stamp = ros::Time::now();
    attitude_setpoint.thrust = 0.5; //this is the thrust that allow a constant altitude no matter what

    accel_target = LQR_to_acceleration(ref);
    attitude_setpoint.orientation = accel_to_orientation(accel_target);
    if(SHOW_PRINTS & time_cout%rate_int==0){
        printf("ref pose\tx %f,\ty %f,\tz %f\n",ref.position.x,
                            ref.position.y, ref.position.z);
    }    

}

void InteractOperation::update_transition_state()
{
// try to make a smooth transition when the relative targeted position between the drone
// and the mast is changed

// Analysis on the x axis
    if (abs(desired_offset.x - transition_state.state.position.x) >= 0.001){
    // if we are in a transition state on the x axis
        transition_state.finished_bitmask &= ~0x1;
        if (Util::sq(transition_state.state.velocity.x) / 2.0 / transition_state.cte_acc 
                            >= abs(desired_offset.x - transition_state.state.position.x))
        {
        // if it is time to brake to avoid overshoot
            //set the transition acceleration (or deceleration) to the one that will lead us to the exact point we want
            transition_state.state.acceleration_or_force.x = - Util::sq(transition_state.state.velocity.x) 
                                            /2.0 / (desired_offset.x - transition_state.state.position.x);
        }
        else if (abs(transition_state.state.velocity.x) > transition_state.max_vel)
        {
        // if we have reached max transitionning speed
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
        transition_state.finished_bitmask = ~0x2;
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
        transition_state.finished_bitmask = ~0x4;
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
    float dist = transition_state.state.position.x - DIST_FH_DRONE_CENTRE; //assuming that the drone is always accurate
    float dist_acc_decc = MAX_VEL*MAX_VEL/MAX_ACCEL;
    if (dist < dist_acc_decc)
        return 2.0 * sqrt(2.0*dist/MAX_ACCEL);
    else
        return (dist - dist_acc_decc) / MAX_VEL //time during max vel
                            + 2 * MAX_VEL / MAX_ACCEL; //time during acceleration and decceleration
}

void InteractOperation::tick() {
    time_cout++;
    mavros_msgs::PositionTarget interact_pt_state = mast.get_interaction_point_state();
    //printf("mast pitch %f, roll %f, angle %f\n", mast_angle.x, mast_angle.y, mast_angle.z);
    // Wait until we get the first module position readings before we do anything else.
    if (interact_pt_state.header.seq == 0) {
        if(time_cout%rate_int==0)
            printf("Waiting for callback\n");
        startApproaching = ros::Time::now();
        return;
    }

    if(SHOW_PRINTS)
        if(time_cout%rate_int == 0)
            ROS_INFO_STREAM("max pitch ETA: " << ros::Time::now() + ros::Duration(mast.time_to_max_pitch()));

    update_transition_state();

    geometry_msgs::Point rotated_offset = rotate(desired_offset,mast.get_yaw());
    const double dx = interact_pt_state.position.x + rotated_offset.x - getCurrentPose().pose.position.x;
    const double dy = interact_pt_state.position.y + rotated_offset.y - getCurrentPose().pose.position.y;
    const double dz = interact_pt_state.position.z + rotated_offset.z - getCurrentPose().pose.position.z;
    const double distance_to_offset = sqrt(Util::sq(dx) + Util::sq(dy) + Util::sq(dz));
    

    switch (interaction_state) {
        case InteractionState::APPROACHING: {
            if (SHOW_PRINTS){
                if(time_cout%rate_int==0) {
                    printf("APPROACHING\t");
                    printf("distance to ref %f\n", distance_to_offset);
                }
            }
            float time_out_gain = 1 + (ros::Time::now()-startApproaching).toSec()/30.0;
            
            
            if(MAST_INTERACT) {
                if ( distance_to_offset <= APPROACH_ACCURACY *time_out_gain ) { 
                    //Todo, we may want to judge the velocity in stead of having a time to completion
                    if (completion_count < ceil(TIME_TO_COMPLETION * (float)rate_int)-1 )
                        completion_count++;
                    else if(mast.time_to_max_pitch() !=-1){
                        //We consider that if the drone is ready at some point, it will 
                        // remain ready until it is time to try
                        completion_count = 0;

                        ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                                << ": Turning on close tracking");
                        //Assuming that the approching state is close enough for close tracking.
                        // send a message to perception to switch close tracking on.
                        //ros::ServiceClient switch_close_tracking = node_handle.serviceClient<fluid::CloseTracking>("perception_main/switch_to_close_tracking");
                        //perception::CloseTracking switch_to_close_tracking_handle;
                        //switch_to_close_tracking_handle.request.timeout = 0.1;

                        //TODO: call the perception_main/switch_to_close_tracking service.
                    
                        float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast();
                        ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                                << ": Control ready to set the FaceHugger. Waiting for the best opportunity"
                                << "\nEstimated waiting time before go: "
                                << time_to_wait);
                        interaction_state = InteractionState::READY;                
                    }
                }
                else
                    completion_count = 0;
            }
            break;
        }
        case InteractionState::READY: {
            //The drone is ready, we just have to wait for the best moment to go!
            float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast();
            if (SHOW_PRINTS){
                if(time_cout%(rate_int/2)==0) {
                    ROS_INFO_STREAM("READY; "
                            << "Estimated waiting time before go: "
                            << time_to_wait);
                }
            }
            close_tracking = true;
            //todo: check the good topic instead
            
            if( abs(time_to_wait) <= TIME_WINDOW_INTERACTION and close_tracking)
            { //We are in the good window to set the faceHugger
              // Notice that once the drone is over, the FH is not set yet.
                interaction_state = InteractionState::OVER;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Approaching -> Over");
                //the offset is set in the frame of the mast:    
                desired_offset.x = DIST_FH_DRONE_CENTRE;  //forward
                desired_offset.y = 0.0;   //left
                desired_offset.z = -0.45; //up
                transition_state.cte_acc = MAX_ACCEL;
                transition_state.max_vel = MAX_VEL;

                // Avoid going to the next step before the transition is actuallized
                transition_state.finished_bitmask = 0;
            }

            break;
        }
        case InteractionState::OVER: {
            if (SHOW_PRINTS){
                if(time_cout%(rate_int*2)==0) printf("OVER\n");
            }
            //We assume that the accuracy is fine, we don't want to take the risk to stay too long
            if (transition_state.finished_bitmask & 0x7 == 0x7) {
                interaction_state = InteractionState::INTERACT;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Over -> Interact");
                desired_offset.x = DIST_FH_DRONE_CENTRE;  //forward
                desired_offset.y = 0.0;   //left
                desired_offset.z -= 0.2;  //up

                // Avoid going to the next step before the transition is actuallized
                transition_state.finished_bitmask = 0;
            }
            break;
        }
        case InteractionState::INTERACT: {
            if (SHOW_PRINTS){
                if(time_cout%(rate_int*2)==0) printf("INTERACT\n");
            }

            // we don't want to take the risk to stay too long, 
            // Whether the faceHugger is set or not, we have to exit.
            // NB, when FH is set, an interupt function switches the state to EXIT
            if (transition_state.finished_bitmask & 0x7 == 0x7) {
                interaction_state = InteractionState::EXIT;
                ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                        << "Interact -> Exiting\n"
                        << "Exit for safety reasons, the FaceHugger could not be placed..."); 

                //we move backward to ensure there will be no colision
                // We directly set the transition state as we want to move as fast as possible
                // and we don't mind anymore about the relative position to the mast
                transition_state.state.position.x = 2.70;  //further than the desired offset as a fix to make it faster
                transition_state.state.position.y = 0.0;   
                transition_state.state.position.z = -1.0;  
                desired_offset.x = 1.70;   //forward
                desired_offset.y = 0.0;    //left
                desired_offset.z = -0.8;   //up

                //todo: for some reason, the drone is slow to get there. 
                // It would be nice to get it go back to a stable approach within 5 secs
                // so that we can try to set the FaecHugger as soon as possible!
                // It may be because of the LQR control which is tuned for accuracy and not fast movements.
                // It can be overriden by setting far setpoints, but that's not clean, and may include risks
            }
            break;
        }
        case InteractionState::EXIT: {
            // NB, when FH is set, an interupt function switches the state to EXIT
            #if SHOW_PRINTS
            if(time_cout%(rate_int*2)==0) printf("EXIT\n");
            #endif
            
            if(transition_state.state.position.x > MAX_DIST_FOR_CLOSE_TRACKING && close_tracking){
            // send a message to AI to switch close tracking off.
                //ros::ServiceClient switch_close_tracking = node_handle.serviceClient<fluid::CloseTracking>("perception_main/switch_to_close_tracking");
                //perception::CloseTracking switch_to_close_tracking_handle;
                //switch_to_close_tracking_handle.request.timeout = 0.1;

                //TODO: call the perception_main/switch_to_close_tracking service.
                close_tracking = false;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "switch close tracking on");
            }

            //This is a transition state before going back to approach and come back the the base or try again.
            if ( distance_to_offset < 0.6 ) {
                if (faceHugger_is_set){
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Extracted");
                    interaction_state = InteractionState::EXTRACTED;
                    desired_offset.x = 2;
                    desired_offset.y = 0.0;
                    desired_offset.z = 3;
                }
                else {
                    //we may want to reset startApproaching, but I don't think so
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Approaching");
                    interaction_state = InteractionState::APPROACHING;
                    desired_offset.x = 1.5;
                    desired_offset.y = 0.0;
                    desired_offset.z = -0.45;
                }
            }
            break;
        }
        case InteractionState::EXTRACTED: {
            #if SHOW_PRINTS
            if(time_cout%(rate_int*2)==0) printf("EXIT\n");
            #endif
            // This is also a transition state before AI takes the lead back and travel back to the starting point
            std_srvs::SetBool request;
            request.request.data = false; 
            if ( distance_to_offset < 0.2 ) {
                desired_offset.x = 2;
                desired_offset.y = 0.0;
                desired_offset.z = 3;
            }
        }
    }//end switch state

    if (time_cout % rate_int == 0)
    {
        if (SHOW_PRINTS){
            printf("transition pose\tx %f,\ty %f,\tz %f\n",transition_state.state.position.x,
                            transition_state.state.position.y, transition_state.state.position.z);
            geometry_msgs::Point cur_drone_pose = getCurrentPose().pose.position;
            printf("Drone pose\tx %f,\ty %f,\tz %f\tyaw %f\n",cur_drone_pose.x,
                                         cur_drone_pose.y, cur_drone_pose.z,getCurrentYaw());
            printf("Accel target\tx %f,\ty %f,\tz %f\n",accel_target.x,
                                         accel_target.y, accel_target.z);
        }
    }

    mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
    update_attitude_input(smooth_rotated_offset);

    if (time_cout % 2 == 0)
    {
        // todo: it may be possible to publish more often without any trouble.
        setpoint.header.stamp = ros::Time::now();
        setpoint.yaw = mast.get_yaw()+M_PI;
        setpoint.position.x = interact_pt_state.position.x + smooth_rotated_offset.position.x;
        setpoint.position.y = interact_pt_state.position.y + smooth_rotated_offset.position.y;
        setpoint.position.z = interact_pt_state.position.z + smooth_rotated_offset.position.z;
        setpoint.velocity.x = interact_pt_state.velocity.x + smooth_rotated_offset.velocity.x;
        setpoint.velocity.y = interact_pt_state.velocity.y + smooth_rotated_offset.velocity.y;
        setpoint.velocity.z = interact_pt_state.velocity.z + smooth_rotated_offset.velocity.z;

        altitude_and_yaw_pub.publish(setpoint);
    }

    attitude_pub.publish(attitude_setpoint);

    #if SAVE_DATA
    //save the control data into files
    reference_state.saveStateLog(Util::addPositionTarget(interact_pt_state, smooth_rotated_offset));
    drone_pose.saveStateLog( getCurrentPose().pose.position,getCurrentTwist().twist.linear,getCurrentAccel());
    #endif

}
