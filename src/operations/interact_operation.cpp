/**
 * @file interact_operation.cpp
 */
#include "interact_operation.h"

#include "mavros_interface.h"
#include "util.h"
#include "fluid.h" //to get access to the tick rate
#include "type_mask.h"

#include <ascend_msgs/SetInt.h>
#include <std_srvs/Trigger.h>

//A list of parameters for the user
#define MAST_INTERACT false //safety feature to avoid going at close proximity to the mast and set the FH
#define MAX_DIST_FOR_CLOSE_TRACKING     1.0 //max distance from the mast before activating close tracking
#define TIME_WINDOW_INTERACTION 1.0 // window within the drone is allowed to go to the OVER state    

// Important distances
#define DIST_FH_DRONE_CENTRE_X   0.42 // 0.5377
#define DIST_FH_DRONE_CENTRE_Y   0.02 // 0.5377
#define DIST_FH_DRONE_CENTRE_Z  -0.4914 //-0.3214

#define SAVE_DATA   true
#define SAVE_Z      false
#define USE_SQRT    false
#define ATTITUDE_CONTROL 4   //4 = ignore yaw rate   //Attitude control does not work without thrust

#define TIME_TO_COMPLETION 0.5 //time in sec during which we want the drone to succeed a state before moving to the other.
#define APPROACH_ACCURACY 0.1 //Accuracy needed by the drone to go to the next state

// Feedforward tuning
#define ACCEL_FEEDFORWARD_X 0.0
#define ACCEL_FEEDFORWARD_Y 0.0

#define MAX_ANGLE   1500 // in centi-degrees 
#define MAX_LQR_ACCEL 1.0 // 0.69m/s2 ~= 4°


uint16_t time_cout = 0; //used not to do some stuffs at every tick

//function called when creating the operation
InteractOperation::InteractOperation(const float& fixed_mast_yaw, const float& offset) : 
            Operation(OperationIdentifier::INTERACT, false, false) { 
    mast = Mast(fixed_mast_yaw);
    
    //get parameters from the launch file.
    const float* temp = Fluid::getInstance().configuration.LQR_gains;
    Kp_LQR = temp[0];
    Kv_LQR = temp[1];
    SHOW_PRINTS = Fluid::getInstance().configuration.interaction_show_prints;
    EKF = Fluid::getInstance().configuration.ekf;
    USE_PERCEPTION = Fluid::getInstance().configuration.use_perception;
    MAX_ACCEL = Fluid::getInstance().configuration.interact_max_acc;
    MAX_VEL = Fluid::getInstance().configuration.interact_max_vel;

    //Choose an initial offset. It is the offset for the approaching state.
    //the offset is set in the frame of the mast:    
    desired_offset.x = offset;     //forward
    desired_offset.y = DIST_FH_DRONE_CENTRE_Y;     //left
    desired_offset.z = DIST_FH_DRONE_CENTRE_Z+0.03;    //up

    }

void InteractOperation::initialize() {

    if(EKF){
        ekf_module_pose_subscriber = node_handle.subscribe("/ekf/module/state",
                                     10, &InteractOperation::ekfModulePoseCallback, this);
        ekf_state_vector_subscriber = node_handle.subscribe("/ekf/state",
                                     10, &InteractOperation::ekfStateVectorCallback, this);
        gt_module_pose_subscriber = node_handle.subscribe("/simulator/module/ground_truth/pose",
                                     10, &InteractOperation::gt_modulePoseCallback, this);
        ROS_INFO_STREAM("/fluid: Uses EKF data");
    }
    else{
    //module_pose_subscriber = node_handle.subscribe("/simulator/module/ground_truth/pose",
    //                                10, &InteractOperation::gt_modulePoseCallback, this);
    module_pose_subscriber = node_handle.subscribe("/model_publisher/module_position",
                                    10, &InteractOperation::gt_modulePoseCallback, this);
    }
    fh_state_subscriber = node_handle.subscribe("/fh_interface/fh_state",
                                    10, &InteractOperation::FaceHuggerCallback, this);
    close_tracking_ready_subscriber = node_handle.subscribe("/close_tracking_running",
                                    10, &InteractOperation::closeTrackingCallback, this);

    start_close_tracking_client = node_handle.serviceClient<ascend_msgs::SetInt>("start_close_tracking");
    pause_close_tracking_client = node_handle.serviceClient<std_srvs::Trigger>("Pause_close_tracking");

    interact_fail_pub = node_handle.advertise<std_msgs::Int16>("/fluid/interact_fail",10);
    
    attitude_pub = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10);
    //creating own publisher to choose exactly when we send messages
    altitude_and_yaw_pub = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);
    attitude_setpoint.type_mask = ATTITUDE_CONTROL;   
    attitude_setpoint.header.frame_id = "interact";   
    
    setpoint.type_mask = TypeMask::POSITION_AND_VELOCITY;
    setpoint.header.frame_id = "interact";

    MavrosInterface mavros_interface;
    mavros_interface.setParam("ANGLE_MAX", MAX_ANGLE);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << MAX_ANGLE/100.0 << " deg.");

    // The transition state is mesured in the mast frame
    transition_state.state.position = desired_offset;
    transition_state.cte_acc = MAX_ACCEL; 
    transition_state.max_vel = MAX_VEL;
    
    faceHugger_is_set = false;    
    close_tracking_is_set = false;
    close_tracking_is_ready = false;

    #if SAVE_DATA
    reference_state = DataFile("reference_state.txt");
    drone_pose = DataFile("drone_pose.txt");
    LQR_input = DataFile("LQR_input.txt");
    gt_reference = DataFile("gt_reference.txt");

    reference_state.shouldSaveZ(SAVE_Z);
    drone_pose.shouldSaveZ(SAVE_Z);
    LQR_input.shouldSaveZ(SAVE_Z);
    gt_reference.shouldSaveZ(SAVE_Z);

    reference_state.initStateLog();
    drone_pose.initStateLog();    
    #if SAVE_Z
    LQR_input.init("Time\tAccel.x\tAccel.y\tAccel.z");
    gt_reference.init("Time\tpose.x\tpose.y\tpose.z");
    #else
    LQR_input.init("Time\tAccel.x\tAccel.y");
    gt_reference.init("Time\tpose.x\tpose.y");
    #endif
    #endif

    //sanity check that the drone is facing the mast.
    ros::Rate rate(rate_int);
    setpoint.position = getCurrentPose().pose.position;
    setpoint.yaw = mast.get_yaw()+M_PI;
    double yaw_err = Util::moduloPi( mast.get_yaw()+M_PI - getCurrentYaw() );
    while( abs( yaw_err) > M_PI/20.0 and ros::ok()){
        altitude_and_yaw_pub.publish(setpoint);
        rate.sleep();
        ros::spinOnce();
        yaw_err = Util::moduloPi( mast.get_yaw()+M_PI - getCurrentYaw() );
    }

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
    mast.set_period(2*M_PI/ekf_state.data[4]);
}

void InteractOperation::gt_modulePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr) {
    #if SAVE_DATA
        mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
        geometry_msgs::Vector3 vec;
        vec.x = module_pose_ptr->pose.pose.position.x + smooth_rotated_offset.position.x;
        vec.y = module_pose_ptr->pose.pose.position.y + smooth_rotated_offset.position.y;
        vec.z = module_pose_ptr->pose.pose.position.z + smooth_rotated_offset.position.z;
        gt_reference.saveVector3(vec);
    #endif
    if(!EKF){
        const geometry_msgs::Vector3 received_eul_angle = Util::quaternion_to_euler_angle(module_pose_ptr->pose.pose.orientation);
        mast.update(module_pose_ptr);
        mast.search_period(received_eul_angle.y); //pitch is y euler angle because of different frame
    }
}

void InteractOperation::FaceHuggerCallback(const std_msgs::Bool released){
    if (released.data && !faceHugger_is_set){
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "CONGRATULATION, FaceHugger set on the mast! We can now exit the mast");
        interaction_state =  InteractionState::EXIT;
        faceHugger_is_set = true;

        desired_offset.x = 2.0;   //forward
        desired_offset.y = DIST_FH_DRONE_CENTRE_Y;    //left
        desired_offset.z = DIST_FH_DRONE_CENTRE_Z - 0.3;   //up
        transition_state.state.position.z = desired_offset.z;
        transition_state.cte_acc = MAX_ACCEL*3;
        transition_state.max_vel = MAX_VEL*3;
        transition_state.finished_bitmask = 0x0;
        //transition_state.state.velocity.x = MAX_VEL*3;
    }
}

void InteractOperation::closeTrackingCallback(std_msgs::Bool ready){
    close_tracking_is_ready = ready.data; 
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
        accel_target.x = Kp_LQR * Util::signed_sqrt(ref.position.x - getCurrentPose().pose.position.x) 
                     + Kv_LQR * Util::signed_sqrt(ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = Kp_LQR * Util::signed_sqrt(ref.position.y - getCurrentPose().pose.position.y) 
                     + Kv_LQR * Util::signed_sqrt(ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;

    #else
        accel_target.x = Kp_LQR * (ref.position.x - getCurrentPose().pose.position.x) 
                     + Kv_LQR * (ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = Kp_LQR * (ref.position.y - getCurrentPose().pose.position.y) 
                     + Kv_LQR * (ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;
    #endif
    // the right of the mast is the left of the drone: the drone is facing the mast
    accel_target.x = - accel_target.x;
    accel_target = rotate(accel_target, mast.get_yaw());// + M_PI);
    return accel_target;
}

geometry_msgs::Quaternion InteractOperation::accel_to_orientation(geometry_msgs::Vector3 accel){
    double yaw = mast.get_yaw() + M_PI; //we want to face the mast
    double roll = atan2(accel.y,9.81);
    double pitch = atan2(accel.x,9.81);
    return Util::euler_to_quaternion(yaw, roll, pitch);
}

void InteractOperation::update_attitude_input(mavros_msgs::PositionTarget ref){
    attitude_setpoint.header.seq++;
    attitude_setpoint.header.stamp = ros::Time::now();
    attitude_setpoint.thrust = 0.5; //this is the thrust that allow a constant altitude no matter what

    accel_target = LQR_to_acceleration(ref);
    if( Util::sq(accel_target.x)+Util::sq(accel_target.y) > Util::sq(MAX_LQR_ACCEL)){
        //constraining the max angle manually because we don't want the drone MAX_ANGLE parameter to be few degrees.
        float temp_angle = atan2(accel_target.y,accel_target.x);
        accel_target.x = MAX_LQR_ACCEL * cos(temp_angle);
        accel_target.y = MAX_LQR_ACCEL * sin(temp_angle);
    }

    //accel_target = rotate(accel_target,getCurrentYaw()-mast.get_yaw());
    
    attitude_setpoint.orientation = accel_to_orientation(accel_target);
    if(SHOW_PRINTS && (time_cout%rate_int)==0){
        printf("ref pose\tx %f,\ty %f,\tz %f\n",ref.position.x,
                            ref.position.y, ref.position.z);
        printf("ref vel\tx %f,\ty %f,\tz %f\n", ref.velocity.x,
                            ref.velocity.y, ref.velocity.z);
    }
}

void InteractOperation::update_transition_state()
{// try to make a smooth transition when the relative targeted position between the drone
// and the mast is changed
//mavros_msgs::PositionTargetConstPtr ts = &transition_state.state;
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
    float dist = transition_state.state.position.x - DIST_FH_DRONE_CENTRE_X; //assuming that the drone is always accurate
    float dist_acc_decc = Util::sq(MAX_VEL)/MAX_ACCEL;
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
            ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                                << ": Waiting for interaction point pose callback\n");
        startApproaching = ros::Time::now();
        return;
    }

    update_transition_state();

    geometry_msgs::Point rotated_offset = rotate(desired_offset,mast.get_yaw());
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
                float time_out_gain = 1 + (ros::Time::now()-startApproaching).toSec()/30.0;
                if ( distance_to_offset <= APPROACH_ACCURACY *time_out_gain ) { 
                    //Todo, we may want to judge the velocity in stead of having a time to completion
                    if (completion_count < ceil(TIME_TO_COMPLETION * (float)rate_int) )
                        completion_count++;
                    else {
                        //We consider that if the drone is ready at some point, it will 
                        //remain ready until it is time to try
                        ROS_INFO_STREAM(ros::this_node::getName().c_str()
                                        << ": " << "Approaching -> Ready");

                        completion_count = 0;
                        ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                                << ": Control ready to set the FaceHugger."
                                << " Waiting for the best opportunity");
                        interaction_state = InteractionState::READY;   
                        desired_offset.x = MAX_DIST_FOR_CLOSE_TRACKING;             
                    }
                }
                else
                    completion_count < 2 ? 0 : completion_count-2; //not reset to 0, but remove 2.
            }
            break;
        }
        case InteractionState::READY: {
            //The drone is ready, we just have to wait for the best moment to go!
            if(!close_tracking_is_set and (transition_state.finished_bitmask & 0x7) == 0x7){
                ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                        << ": Turning on close tracking");
                
                // send a message to perception to switch close tracking on.
                if(USE_PERCEPTION){
                    ascend_msgs::SetInt srv;
                    srv.request.data = 10;
                    if (start_close_tracking_client.call(srv)){
                        close_tracking_is_set = true; 
                    }
                }
                else{
                    close_tracking_is_set= true; //Todo, to be removed
                    close_tracking_is_ready = true;
                }
            }
            if(mast.time_to_max_pitch() !=-1){ //we don't konw it yet
                float time_to_wait = mast.time_to_max_pitch()-estimate_time_to_mast();
//              printf("t_2max_pitch %f\t t_2mast %f\tt_wait %f\n",
//                          mast.time_to_max_pitch(), estimate_time_to_mast(), time_to_wait);
                if(time_to_wait < -TIME_WINDOW_INTERACTION){
                    time_to_wait+=mast.get_period();
                }
                if (SHOW_PRINTS and time_cout%(rate_int/2)==0) {
                    ROS_INFO_STREAM("READY; "
                            << "Estimated waiting time before go: "
                            << time_to_wait);
                }
                if( close_tracking_is_ready and (abs(time_to_wait) <= TIME_WINDOW_INTERACTION) )
                { //We are in the good window to set the faceHugger
                    interaction_state = InteractionState::OVER;
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                                << ": " << "Ready -> Over");
                    desired_offset.x = DIST_FH_DRONE_CENTRE_X;  //forward
                    desired_offset.y = DIST_FH_DRONE_CENTRE_Y;   //left
                    desired_offset.z = DIST_FH_DRONE_CENTRE_Z+0.03; //up
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
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Over -> Interact");
                desired_offset.x = DIST_FH_DRONE_CENTRE_X;  //forward
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
                ROS_INFO_STREAM(ros::this_node::getName().c_str() 
                        << "Interact -> Exiting\n"
                        << "Exit for safety reasons, the FaceHugger could not be placed..."); 

                //we move backward to ensure there will be no colision
                // We directly set the transition state as we want to move as fast as possible
                // and we don't mind anymore about the relative position to the mast
                desired_offset.x = 2;   //forward
                desired_offset.y = DIST_FH_DRONE_CENTRE_Y;    //left
                desired_offset.z = DIST_FH_DRONE_CENTRE_Z;   //up
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
                if(USE_PERCEPTION){ //we are getting to far from the mast, and the position is not stable.
                    std_srvs::Trigger srv;
                    if (pause_close_tracking_client.call(srv)){
                        close_tracking_is_set = false;
                    }
                }
                else{
                        close_tracking_is_set = false;
                        close_tracking_is_ready = false;
                }
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                        << ": " << "switch close tracking off");
            }
            
            // Come back the the base or try again.
            if ( distance_to_offset < 0.2 ) {
                if (faceHugger_is_set){
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Extracted");
                    interaction_state = InteractionState::EXTRACTED;
                    desired_offset.x = 4;
                    desired_offset.y = DIST_FH_DRONE_CENTRE_Y;
                    desired_offset.z = 3;
                    transition_state.cte_acc = MAX_ACCEL*3;
                    transition_state.max_vel = MAX_VEL*3;
                }
                else {
                    ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Exit -> Approaching");
                    //ascend_msgs::SetInt interact_fail_srv;
                    number_fail.data++;
                    //interact_fail_srv.request.data = number_fail;
                    for(int i = 0; i<3 ; i ++) interact_fail_pub.publish(number_fail);
                    interaction_state = InteractionState::APPROACHING;
                    desired_offset.x = 2;
                    desired_offset.y = DIST_FH_DRONE_CENTRE_Y;
                    desired_offset.z = DIST_FH_DRONE_CENTRE_Z+0.03;
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
//        printf("transition pose\tx %f,\ty %f,\tz %f\n",transition_state.state.position.x,
//                        transition_state.state.position.y, transition_state.state.position.z);
        geometry_msgs::Point cur_drone_pose = getCurrentPose().pose.position;
        printf("Drone pose\tx %f,\ty %f,\tz %f\tyaw %f\n",cur_drone_pose.x,
                                        cur_drone_pose.y, cur_drone_pose.z,getCurrentYaw());
        printf("Accel target\tx %f,\ty %f,\tz %f\n",accel_target.x,
                                        accel_target.y, accel_target.z);
    }

    mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,mast.get_yaw());
    mavros_msgs::PositionTarget ref = Util::addPositionTarget(interact_pt_state,smooth_rotated_offset);
    update_attitude_input(ref);

    if (time_cout % 2 == 0) {
        // todo: it may be possible to publish more often without any trouble.
        setpoint.header.seq++;
        setpoint.header.stamp = ros::Time::now();
        setpoint.yaw = mast.get_yaw()+M_PI;
        setpoint.position = ref.position;
        setpoint.velocity = ref.velocity;

        //altitude_and_yaw_pub.publish(setpoint);
    }

    attitude_pub.publish(attitude_setpoint);

    #if SAVE_DATA
        reference_state.saveStateLog(ref);
        geometry_msgs::Vector3 drone_acc = rotate(getCurrentAccel(),mast.get_yaw()+M_PI);
        drone_acc.x = -drone_acc.x;
        drone_pose.saveStateLog( getCurrentPose().pose.position,getCurrentTwist().twist.linear,drone_acc);
        accel_target.y = - accel_target.y;
        LQR_input.saveVector3(accel_target);

    #endif
}
