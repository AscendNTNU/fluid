/**
 * @file extract_module_operation.cpp
 */
#include "extract_module_operation.h"

#include "mavros_interface.h"
#include <mavros_msgs/AttitudeTarget.h>
#include "util.h"

#include <std_srvs/SetBool.h>

//includes to write in a file
#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory
#include "fluid.h" //to get access to the tick rate

//A list of parameters for the user
#define SAVE_DATA   true
#define SHOW_PRINTS false
#define SAVE_Z      true
#define USE_SQRT    false
#define ATTITUDE_CONTROL 0      //Attitude control does not work without thrust
#define POS_AND_VEL_CONTROL 2496 //typemask for setpoint_raw/local


// LQR tuning
#define POW          1.0
#define RATION       2.84       // =0.37 / 0.13
#define LQR_POS_GAIN 0.4189     //tuned for 0.13m pitch radius and 0.37m roll radius
#define LQR_VEL_GAIN 1.1062
#define ACCEL_FEEDFORWARD_X 0.0
#define ACCEL_FEEDFORWARD_Y 0.0
const float K_LQR_X[2] = {POW*LQR_POS_GAIN, POW*LQR_VEL_GAIN};
const float K_LQR_Y[2] = {RATION*POW*LQR_POS_GAIN, RATION*POW*LQR_VEL_GAIN};

//smooth transition
#define MAX_VEL     0.14
#define MAX_ACCEL   0.07

#define MAX_ANGLE   400 // in centi-degrees


#if SAVE_DATA
//files saved in home directory
const std::string reference_state_path = std::string(get_current_dir_name()) + "/../reference_state.txt";
const std::string drone_pose_path      = std::string(get_current_dir_name()) + "/../drone_state.txt";    
const std::string drone_setpoints_path = std::string(get_current_dir_name()) + "/../drone_setpoints.txt";
std::ofstream reference_state_f; 
std::ofstream drone_pose_f; 
std::ofstream drone_setpoints_f; 
bool store_data = true;
#endif

uint8_t time_cout = 0;

//todo: this function should probably be place in Util
double sq(double n) {return n*n;}

//todo: this function should probably be place in Util
mavros_msgs::PositionTarget addState(mavros_msgs::PositionTarget a, mavros_msgs::PositionTarget b){
    mavros_msgs::PositionTarget res;
    res.header = a.header; // this is arbitrary. Did no find a perfect solution, but should not have any impact

    res.position.x = a.position.x + b.position.x;
    res.position.y = a.position.y + b.position.y;
    res.position.z = a.position.z + b.position.z;

    res.velocity.x = a.velocity.x + b.velocity.x;
    res.velocity.y = a.velocity.y + b.velocity.y;
    res.velocity.z = a.velocity.z + b.velocity.z;

    res.acceleration_or_force.x = a.acceleration_or_force.x + b.acceleration_or_force.x;
    res.acceleration_or_force.y = a.acceleration_or_force.y + b.acceleration_or_force.y;
    res.acceleration_or_force.z = a.acceleration_or_force.z + b.acceleration_or_force.z;

    return res;
}

//todo: this function should probably be place in Util
double signed_sqrt(double nb){
    return nb>0 ? sqrt(nb) : -sqrt(-nb);
}
//todo: add that again
#if SAVE_DATA
void ExtractModuleOperation::initLog(const std::string file_name)
{ //create a header for the logfile.
    std::ofstream save_file_f;
     save_file_f.open(file_name);
    if(save_file_f.is_open())
    {
//        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": " << file_name << " open successfully");
        save_file_f << "Time\tPos.x\tPos.y\tPos.z\tVel.x\tVel.y\tVel.z\tmodule_estimate_vel.x\tmodule_estimate_vel.y\tmodule_estimate_vel.z\n";
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << file_name);
        store_data = false; //if we can't save data, we don't want to do it
    }
}

void ExtractModuleOperation::saveLog(const std::string file_name, const geometry_msgs::PoseStamped pose, const geometry_msgs::TwistStamped vel, const geometry_msgs::Vector3 accel)
{
    if(store_data)
    {
        std::ofstream save_file_f;
        save_file_f.open (file_name, std::ios::app);
        if(save_file_f.is_open())
        {
            save_file_f << std::fixed << std::setprecision(3) //only 3 decimals
                            << ros::Time::now() << "\t"
                            << pose.pose.position.x << "\t"
                            << pose.pose.position.y << "\t"
                            #if SAVE_Z
                            << pose.pose.position.z << "\t"
                            #endif
                            << vel.twist.linear.x << "\t"
                            << vel.twist.linear.y << "\t"
                            #if SAVE_Z
                            << vel.twist.linear.z << "\t"
                            #endif
                            << accel.x << "\t"
                            << accel.y 
                            #if SAVE_Z
                            << "\t" << accel.z
                            #endif
                            << "\n";
            save_file_f.close();
        }
    }
}

void ExtractModuleOperation::saveLog(const std::string file_name, const mavros_msgs::PositionTarget data)
{
    if(store_data)
    {
        std::ofstream save_file_f;
        save_file_f.open (file_name, std::ios::app);
        if(save_file_f.is_open())
        {
            save_file_f << std::fixed << std::setprecision(3) //only 3 decimals
                            << ros::Time::now() << "\t"
                            << data.position.x << "\t"
                            << data.position.y << "\t"
                            #if SAVE_Z
                            << data.position.z << "\t"
                            #endif
                            << data.velocity.x << "\t"
                            << data.velocity.y << "\t"
                            #if SAVE_Z
                            << data.velocity.z << "\t"
                            #endif
                            << data.acceleration_or_force.x << "\t"
                            << data.acceleration_or_force.y 
                            #if SAVE_Z
                            << "\t" << data.acceleration_or_force.z
                            #endif
                            << "\n";
            save_file_f.close();
        }
    }
}
#endif

ExtractModuleOperation::ExtractModuleOperation(float mast_yaw) : Operation(OperationIdentifier::EXTRACT_MODULE, false) { //function called at the when initiating the operation
    module_pose_subscriber =
        node_handle.subscribe("/simulator/module/ground_truth/pose", 10, &ExtractModuleOperation::modulePoseCallback, this);
    module_pose_subscriber_old =
        node_handle.subscribe("/simulator/module_pose", 10, &ExtractModuleOperation::modulePoseCallback, this);
    backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");
    attitude_pub = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10); //the published topic of the setpoint is redefined
    altitude_and_yaw_pub = node_handle.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10); //the published topic of the setpoint is redefined
    attitude_setpoint.type_mask = ATTITUDE_CONTROL;
    fixed_mast_yaw = mast_yaw;
    tick_rate = (float) Fluid::getInstance().configuration.refresh_rate;
    ROS_INFO_STREAM(ros::this_node::getName().c_str() 
            << ": mast yaw received: " << fixed_mast_yaw);
    //TODO: find a way to desactivate the position setpoints we are normaly using.
}

void ExtractModuleOperation::initialize() {
    MavrosInterface mavros_interface;
    mavros_interface.setParam("ANGLE_MAX", MAX_ANGLE);
    ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat max angle to: " << MAX_ANGLE/100.0 << " deg.");

    // Use the current position as setpoint until we get a message with the module position
    setpoint.position = getCurrentPose().pose.position;
        
    // Entering smooth zone. We could also durectly place the transition_state to 
    // the initial desired offset if we are not scared of hitting the mast.
    // That could save some time without taking risks (?) 

    // The desired offset is mesured in the masr frame (x to the right, y forward, and z upward)
    transition_state.state.position.x = getCurrentPose().pose.position.x - module_state.position.x;
    transition_state.state.position.y = getCurrentPose().pose.position.y - module_state.position.y;
    transition_state.state.position.z = getCurrentPose().pose.position.z - module_state.position.z;

    //the offset is set in the frame of the mast:    
    desired_offset.x = 3.0;   //forward   //right
    desired_offset.y = 0.0;   //left   //front
    desired_offset.z = -0.5; //up   //up
    transition_state.cte_acc = 3*MAX_ACCEL;
    transition_state.max_vel = 3*MAX_VEL;

    #if SAVE_DATA
    //create a header for the datafiles.
    initLog(reference_state_path); 
    initLog(drone_pose_path); 
    initLog(drone_setpoints_path); 
    #endif
}

bool ExtractModuleOperation::hasFinishedExecution() const { return extraction_state == ExtractionState::EXTRACTED; }

void ExtractModuleOperation::modulePoseCallback(
    const geometry_msgs::PoseStampedConstPtr module_pose_ptr) {
    previous_module_state = module_state;

    module_state.header = module_pose_ptr->header;
    module_state.position = module_pose_ptr->pose.position;
    //module_state.velocity = derivate(module_state.position,previous_module_state.position,module_state.header.stamp - previous_module_state.header.stamp);
    //module_state.acceleration_or_force = derivate(module_state.velocity,previous_module_state.velocity,module_state.header.stamp - previous_module_state.header.stamp);
    module_state.velocity = estimateModuleVel();    
    module_state.acceleration_or_force = estimateModuleAccel();
}

geometry_msgs::Vector3 ExtractModuleOperation::estimateModuleVel(){
    // estimate the velocity of the module by simply derivating the position.
    // In the long run, I expect to receive a nicer estimate by perception or to createa KF myself.
    geometry_msgs::Vector3 vel;
    double dt = (module_state.header.stamp - previous_module_state.header.stamp).nsec/1000000000.0;
    // TODO: check that the sign if the output vel is correct
    vel.x = (module_state.position.x - previous_module_state.position.x)/dt;
    vel.y = (module_state.position.y - previous_module_state.position.y)/dt;
    vel.z = (module_state.position.z - previous_module_state.position.z)/dt;
    return vel;
}

geometry_msgs::Vector3 ExtractModuleOperation::estimateModuleAccel(){
    // estimate the velocity of the module by simply derivating the position.
    // In the long run, I expect to receive a nicer estimate by perception or to createa KF myself.
    geometry_msgs::Vector3 Accel;
    double dt = (module_state.header.stamp - previous_module_state.header.stamp).nsec/1000000000.0;
    // TODO: check that the sign if the output vel is correct
    Accel.x = (module_state.velocity.x - previous_module_state.velocity.x)/dt;
    Accel.y = (module_state.velocity.y - previous_module_state.velocity.y)/dt;
    Accel.z = (module_state.velocity.z - previous_module_state.velocity.z)/dt;
    return Accel;
}

geometry_msgs::Vector3 ExtractModuleOperation::derivate(geometry_msgs::Vector3 actual, geometry_msgs::Vector3 last, ros::Time dt_ros){
    geometry_msgs::Vector3 res;
    double dt = dt_ros.nsec/1000000000.0;
    // TODO: check that the sign if the output vel is correct
    res.x = (actual.x - last.x)/dt;
    res.y = (actual.y - last.y)/dt;
    res.z = (actual.z - last.z)/dt;
    return res;
}

//todo: this function should probably be place in Util
mavros_msgs::PositionTarget ExtractModuleOperation::rotate(mavros_msgs::PositionTarget setpoint, float yaw){
    mavros_msgs::PositionTarget rotated_setpoint;
    rotated_setpoint.position = rotate(setpoint.position);
    rotated_setpoint.velocity = rotate(setpoint.velocity);
    rotated_setpoint.acceleration_or_force = rotate(setpoint.acceleration_or_force);

    return rotated_setpoint;
}

//todo: this function should probably be place in Util
geometry_msgs::Vector3 ExtractModuleOperation::rotate(geometry_msgs::Vector3 pt, float yaw){
    geometry_msgs::Vector3 rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

//todo: this function should probably be place in Util
geometry_msgs::Point ExtractModuleOperation::rotate(geometry_msgs::Point pt, float yaw){
    geometry_msgs::Point rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

void ExtractModuleOperation::LQR_to_acceleration(mavros_msgs::PositionTarget ref, bool use_sqrt){
    accel_target.z = 0;
    if (!use_sqrt)
    {
        accel_target.x = K_LQR_X[0] * (ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * (ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_X[0] * (ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_X[1] * (ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;
    }
    else
    {
        accel_target.x = K_LQR_X[0] * signed_sqrt(ref.position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * signed_sqrt(ref.velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.x;
        accel_target.y = K_LQR_X[0] * signed_sqrt(ref.position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_X[1] * signed_sqrt(ref.velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref.acceleration_or_force.y;
    }
    //taking the opposite of the acc because the drone is facing the mast
    // the right of the mast is the left of the drone
    accel_target.x = - accel_target.x;
}

//todo: this function should probably be place in Util
geometry_msgs::Quaternion ExtractModuleOperation::euler_to_quaternion(double yaw, double roll, double pitch){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

//todo: this function should probably be place in Util
geometry_msgs::Quaternion ExtractModuleOperation::accel_to_orientation(geometry_msgs::Point accel){
    double yaw = fixed_mast_yaw + M_PI; //we want to face the mast
    double roll = atan2(accel.x,9.81);
    double pitch = atan2(accel.y,9.81);
    return euler_to_quaternion(yaw, pitch, roll);
}

void ExtractModuleOperation::update_attitude_input(mavros_msgs::PositionTarget module,mavros_msgs::PositionTarget offset, bool use_sqrt){
    mavros_msgs::PositionTarget ref = addState(module, offset);

    attitude_setpoint.header.stamp = ros::Time::now();
    attitude_setpoint.thrust = 0.5; //this is the thrust that allow a contanst altitude no matter what

    LQR_to_acceleration(ref, use_sqrt);
    attitude_setpoint.orientation = accel_to_orientation(accel_target);
}

void ExtractModuleOperation::update_transition_state()
{
//try to make a smooth transition when the relative targeted position between the drone
//and the mast is changed

// Analysis on the x axis
    if (abs(desired_offset.x - transition_state.state.position.x) > 0.001){
    // if we are in a transition state on the x axis
        transition_state.finished_bitmask &= ~0x1;
        if (sq(transition_state.state.velocity.x) / 2.0 / transition_state.cte_acc >= abs(desired_offset.x - transition_state.state.position.x))
        {
        // if it is time to brake to avoid overshoot
            //set the transition acceleration (or deceleration) to the one that will lead us to the exact point we want
            transition_state.state.acceleration_or_force.x = - sq(transition_state.state.velocity.x) /2.0 / (desired_offset.x - transition_state.state.position.x);
        }
        else if (abs(transition_state.state.velocity.x) > transition_state.max_vel)
        {
        // if we have reached max transitionning speed
            //we stop accelerating and maintain speed
            transition_state.state.acceleration_or_force.x = 0.0;
            //transition_state.state.velocity = signe(transition_state.state.velocity) * transition_state.state.velocity //to set the speed to the exact chosen value
        }
        else{
        //we are in the acceleration phase of the transition){
            if (desired_offset.x - transition_state.state.position.x > 0.0)
                transition_state.state.acceleration_or_force.x = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.x = - transition_state.cte_acc;
        }
        // Whatever the state we are in, update velocity and position of the target
        transition_state.state.velocity.x = transition_state.state.velocity.x + transition_state.state.acceleration_or_force.x / tick_rate;
        transition_state.state.position.x = transition_state.state.position.x + transition_state.state.velocity.x / tick_rate;
        
    }
    else if (abs(transition_state.state.velocity.x) < 0.1){
        //setpoint reached destination on this axis
        //todo: should the acceleration be tested?            
        transition_state.state.position.x = desired_offset.x;
        transition_state.state.velocity.x = 0.0;
        transition_state.state.acceleration_or_force.x = 0.0;
        transition_state.finished_bitmask |= 0x1;
    }

// Analysis on the y axis, same as on the x axis
    if (abs(desired_offset.y - transition_state.state.position.y) > 0.001){
        transition_state.finished_bitmask = ~0x2;
        if (sq(transition_state.state.velocity.y) / 2.0 / transition_state.cte_acc >= abs(desired_offset.y - transition_state.state.position.y))
            transition_state.state.acceleration_or_force.y = - sq(transition_state.state.velocity.y) /2.0 / (desired_offset.y - transition_state.state.position.y);
        else if (abs(transition_state.state.velocity.y) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.y = 0.0;
        else{
            if (desired_offset.y - transition_state.state.position.y > 0.0)
                transition_state.state.acceleration_or_force.y = transition_state.cte_acc;
            else
                transition_state.state.acceleration_or_force.y = - transition_state.cte_acc;
            }
        transition_state.state.velocity.y  =   transition_state.state.velocity.y  + transition_state.state.acceleration_or_force.y / tick_rate;
        transition_state.state.position.y =  transition_state.state.position.y  + transition_state.state.velocity.y / tick_rate;
    }
    else if (abs(transition_state.state.velocity.y) < 0.1){
        transition_state.state.position.y = desired_offset.y;
        transition_state.state.velocity.y = 0.0;
        transition_state.state.acceleration_or_force.y = 0.0;
        transition_state.finished_bitmask |= 0x2;
    }

// Analysis on the z axis, same as on the x axis
    if (abs(desired_offset.z - transition_state.state.position.z) > 0.001){
        transition_state.finished_bitmask = ~0x4;
        if (sq(transition_state.state.velocity.z) / 2.0 / transition_state.cte_acc >= abs(desired_offset.z - transition_state.state.position.z))
            transition_state.state.acceleration_or_force.z = - sq(transition_state.state.velocity.z) /2.0 / (desired_offset.z - transition_state.state.position.z);
        else if (abs(transition_state.state.velocity.z) > transition_state.max_vel)
            transition_state.state.acceleration_or_force.z = 0.0;
        else {
            if (desired_offset.z - transition_state.state.position.z > 0.0)
                transition_state.state.acceleration_or_force.z = transition_state.cte_acc;
            else 
                transition_state.state.acceleration_or_force.z = - transition_state.cte_acc;
        }
        transition_state.state.velocity.z  =   transition_state.state.velocity.z  + transition_state.state.acceleration_or_force.z / tick_rate;
        transition_state.state.position.z =  transition_state.state.position.z  + transition_state.state.velocity.z / tick_rate;
    }
    else if (abs(transition_state.state.velocity.z) < 0.1){
        transition_state.state.position.z = desired_offset.z;
        transition_state.state.velocity.z = 0.0;
        transition_state.state.acceleration_or_force.z = 0.0;
        transition_state.finished_bitmask |= 0x4;
    }
    
//    if (comparPoints(desired_offset,transition_state.state.position,0.001) and not transition_state.transition_finished){
//        transition_state.transition_finished = true;
//        printf ("transition finished!");
//    }

}

void ExtractModuleOperation::tick() {
    time_cout++;
    // Wait until we get the first module position readings before we do anything else.
    if (module_state.header.seq == 0) {
        printf("waiting for callback\n");
        return;
    }
    
    const double dx = module_state.position.x + desired_offset.x - getCurrentPose().pose.position.x;
    const double dy = module_state.position.y + desired_offset.y - getCurrentPose().pose.position.y;
    const double dz = module_state.position.z + desired_offset.z - getCurrentPose().pose.position.z;

    const double distance_to_reference_with_offset = sqrt(sq(dx) + sq(dy) + sq(dz));
    

    switch (extraction_state) {
        case ExtractionState::APPROACHING: {
            if(time_cout%((int)tick_rate*2)==0) printf("APPROACHING\n");

            if (distance_to_reference_with_offset < 0.04) {
                extraction_state = ExtractionState::OVER;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Approaching -> Over");
                //the offset is set in the frame of the mast:    
                desired_offset.x = 0.25;  //forward   //right //the distance from the drone to the FaceHugger
                desired_offset.y = 0.0;   //left   //front
                desired_offset.z = -0.45;  //up   //up
                transition_state.cte_acc = MAX_ACCEL;
                transition_state.max_vel = MAX_VEL;


            }
            break;
        }
        case ExtractionState::OVER: {
            if(time_cout%((int)tick_rate*2)==0) printf("OVER\n");
            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            //todo write a smart evalutation function to know when to move to the next state
            if (distance_to_reference_with_offset < 0.02 && std::abs(getCurrentYaw() - fixed_mast_yaw) < M_PI / 50.0) {
                extraction_state = ExtractionState::EXTRACTING;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": " << "Over -> Extracting");
                desired_offset.x = 0.25;  //forward   //right //the distance from the drone to the FaceHugger
                desired_offset.y = 0.0;   //left      //front
                desired_offset.z = -0.8;  //up        //up

            }
            
            break;
        }
        case ExtractionState::EXTRACTING: {
            if(time_cout%((int)tick_rate*2)==0) printf("EXTRACTING\n");
            //Do something to release the FaceHugger at the righ moment
            /*
            if (!called_backpropeller_service) {
                std_srvs::SetBool request;
                request.request.data = true;
                backpropeller_client.call(request);
                called_backpropeller_service = true;
            } */          

            // If the module is on the way down
            // TODO: this should be checked in a better way
            if (module_state.position.z < 0.5) {
                extraction_state = ExtractionState::EXTRACTED;
                ROS_INFO_STREAM(ros::this_node::getName().c_str() << "Module extracted!"); 
                std_srvs::SetBool request;
                request.request.data = false;
//                backpropeller_client.call(request);
//                called_backpropeller_service = false;

                //we move backward to ensure there will be no colision
                desired_offset.x = 1.70;  //forward   //right //the distance from the drone to the FaceHugger
                desired_offset.y = 0.0;   //left      //front
                desired_offset.z = -0.8;  //up        //up

            }

            break;
        }
    }//end switch state
    update_transition_state();
    mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,fixed_mast_yaw);

    if (time_cout % ((int)tick_rate) == 0)
    {
    //    printf("desired offset \t\tx %f, y %f, z %f\n",desired_offset.x,
    //                    desired_offset.y, desired_offset.z);
        printf("transition state\t x %f, y %f, z %f\n",transition_state.state.position.x,
                        transition_state.state.position.y, transition_state.state.position.z);
    //    printf("transition vel\t x %f, y %f, z %f\n",transition_state.state.velocity.x,
    //                    transition_state.state.velocity.y, transition_state.state.velocity.z);
    //    printf("transition accel\t x %f, y %f, z %f\n",transition_state.state.acceleration_or_force.x,
    //                    transition_state.state.acceleration_or_force.y, transition_state.state.acceleration_or_force.z);
    }
    if (time_cout % 5 == 0)
    {
        setpoint.yaw = fixed_mast_yaw+M_PI;
        setpoint.position.x = module_state.position.x + smooth_rotated_offset.position.x;
        setpoint.position.y = module_state.position.y + smooth_rotated_offset.position.y;
        setpoint.position.z = module_state.position.z + smooth_rotated_offset.position.z;
        setpoint.velocity.x = module_state.velocity.x + smooth_rotated_offset.velocity.x;
        setpoint.velocity.y = module_state.velocity.y + smooth_rotated_offset.velocity.y;
        setpoint.velocity.z = module_state.velocity.z + smooth_rotated_offset.velocity.z;
        setpoint.type_mask = POS_AND_VEL_CONTROL;
        setpoint.header.stamp = ros::Time::now();

        altitude_and_yaw_pub.publish(setpoint);

    }
    update_attitude_input(module_state,smooth_rotated_offset, USE_SQRT);

    attitude_pub.publish(attitude_setpoint);

    #if SAVE_DATA
    //save the control data into files
    saveLog(reference_state_path,addState(module_state, smooth_rotated_offset));
    saveLog(drone_pose_path, getCurrentPose(),getCurrentTwist(),getCurrentAccel());
    saveLog(drone_setpoints_path,smooth_rotated_offset);
    #endif

}
