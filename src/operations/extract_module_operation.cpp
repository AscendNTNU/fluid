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

//A list of parameters for the user
#define SAVE_DATA   true
#define SAVE_Z      false
#define USE_SQRT    false
#define CONTROL_TYPE 0      //Attitude control does not work without thrust

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
#define MAX_VEL     0.05
#define MAX_ACCEL   0.03

#define MAX_ACCEL_X 0.15 //todo, implement that properly as a drone parameter
#define MAX_ACCEL_Y 0.30

const std::string reference_state_path = std::string(get_current_dir_name()) + "/../reference_state.txt";   //file saved in home
const std::string drone_pose_path = std::string(get_current_dir_name()) + "/../drone_state.txt";            //file saved in home
const std::string drone_setpoints_path = std::string(get_current_dir_name()) + "/../drone_setpoints.txt";   //file saved in home
std::ofstream reference_state_f; 
std::ofstream drone_pose_f; 
std::ofstream drone_setpoints_f; 



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

double signed_sqrt(double nb){
    return nb>0 ? sqrt(nb) : -sqrt(-nb);
}
//todo: add that again
/*void ExtractModuleOperation::initLog()
{ //create a header for the logfile.
    save_drone_position_f.open(logFileName);
    if(save_drone_position_f.is_open())
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": " << logFileName << " open successfully");
        save_drone_position_f << "Time\tPos.x\tPos.y\tPos.z\tVel.x\tVel.y\tVel.z\tmodule_estimate_vel.x\tmodule_estimate_vel.y\tmodule_estimate_vel.z\n";
        save_drone_position_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << logFileName);
        store_data = false; //if we can't save data, we don't want to do it
    }
}

void ExtractModuleOperation::saveLog()
{
    if(store_data)
    {
        save_drone_position_f.open (logFileName, std::ios::app); //stored in fluid directory
        if(save_drone_position_f.is_open())
        {
            save_drone_position_f << std::fixed << std::setprecision(3) //only 3 decimals
                            << ros::Time::now() << "\t"
                            << getCurrentPose().pose.position.x << "\t"
                            << getCurrentPose().pose.position.y << "\t"
                            << getCurrentPose().pose.position.z << "\t"
                            << getCurrentTwist().twist.linear.x << "\t"
                            << getCurrentTwist().twist.linear.y << "\t"
                            << getCurrentTwist().twist.linear.z << "\t"
                            << module_calculated_velocity.x << "\t"
                            << module_calculated_velocity.y << "\t"
                            << module_calculated_velocity.z
                            << "\n";
            save_drone_position_f.close();
        }
    }
}
*/
ExtractModuleOperation::ExtractModuleOperation(float mast_yaw) : Operation(OperationIdentifier::EXTRACT_MODULE, false) { //function called at the when initiating the operation
    module_pose_subscriber =
        node_handle.subscribe("/sim/module_position", 10, &ExtractModuleOperation::modulePoseCallback, this);
    backpropeller_client = node_handle.serviceClient<std_srvs::SetBool>("/airsim/backpropeller");
    attitude_pub = node_handle.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",10); //the published topic of the setpoint is redefined
    attitude_setpoint.type_mask = CONTROL_TYPE;
    fixed_mast_yaw = mast_yaw;

    //TODO: find a way to desactivate the position setpoints we are normaly using.
}

void ExtractModuleOperation::initialize() {
    MavrosInterface mavros_interface;
    //mavros_interface.setParam("WPNAV_SPEED", speed);
    //ROS_INFO_STREAM(ros::this_node::getName().c_str() << ": Sat speed to: " << speed);

    //mavros_interface.setParam("MPC_TILTMAX_AIR", 20);
    //mavros_interface.setParam("MPC_Z_VEL_MAX_DN", 0.5);

    // Use the current position as setpoint until we get a message with the module position
    setpoint.position = getCurrentPose().pose.position;
    
    transition_state.max_vel = MAX_VEL;
    transition_state.cte_acc = MAX_ACCEL;
    
    // Entering smooth zone. We could also durectly place the transition_state to 
    // the initial desired offset if we are not scared of hitting the mast.
    // That could save some time without taking risks (?) 

    // The desired offset is mesured in the masr frame (x to the right, y forward, and z upward)
    desired_offset.x = 0.0;
    desired_offset.y = 1.0;
    desired_offset.z = 0.05;
    transition_state.state.position.x = getCurrentPose().pose.position.x;
    transition_state.state.position.y = getCurrentPose().pose.position.y;
    transition_state.state.position.z = getCurrentPose().pose.position.z;

    //initLog(); //create a header for the logfile.
}

bool ExtractModuleOperation::hasFinishedExecution() const { return extraction_state == ExtractionState::EXTRACTED; }

void ExtractModuleOperation::modulePoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr module_pose_ptr) {
    previous_module_state = module_state;

    module_state.header.stamp = ros::Time::now();
    module_state.position = module_pose_ptr->pose.pose.position;
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

mavros_msgs::PositionTarget ExtractModuleOperation::rotate(mavros_msgs::PositionTarget setpoint, float yaw){
    mavros_msgs::PositionTarget rotated_setpoint;
    rotated_setpoint.position = rotate(setpoint.position);
    rotated_setpoint.velocity = rotate(setpoint.velocity);
    rotated_setpoint.acceleration_or_force = rotate(setpoint.acceleration_or_force);

    return rotated_setpoint;
}

geometry_msgs::Vector3 ExtractModuleOperation::rotate(geometry_msgs::Vector3 pt, float yaw){
    geometry_msgs::Vector3 rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

geometry_msgs::Point ExtractModuleOperation::rotate(geometry_msgs::Point pt, float yaw){
    geometry_msgs::Point rotated_point;
    rotated_point.x = cos(fixed_mast_yaw) * pt.x - sin(fixed_mast_yaw) * pt.y;
    rotated_point.y = cos(fixed_mast_yaw) * pt.y + sin(fixed_mast_yaw) * pt.x;
    rotated_point.z = pt.z;

    return rotated_point;
}

void ExtractModuleOperation::LQR_to_acceleration(mavros_msgs::PositionTarget* ref, geometry_msgs::PointPtr accel_targ, bool use_sqrt){
    accel_targ->z = 0;
    if (!use_sqrt)
    {
        accel_targ->x = K_LQR_X[0] * (ref->position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * (ref->velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref->acceleration_or_force.x;
        accel_targ->y = K_LQR_X[0] * (ref->position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_X[1] * (ref->velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref->acceleration_or_force.y;
    }
    else
    {
        accel_targ->x = K_LQR_X[0] * signed_sqrt(ref->position.x - getCurrentPose().pose.position.x) 
                     + K_LQR_X[1] * signed_sqrt(ref->velocity.x - getCurrentTwist().twist.linear.x) 
                     + ACCEL_FEEDFORWARD_X * ref->acceleration_or_force.x;
        accel_targ->y = K_LQR_X[0] * signed_sqrt(ref->position.y - getCurrentPose().pose.position.y) 
                     + K_LQR_X[1] * signed_sqrt(ref->velocity.y - getCurrentTwist().twist.linear.y) 
                     + ACCEL_FEEDFORWARD_X * ref->acceleration_or_force.y;
    }

}

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

geometry_msgs::Quaternion ExtractModuleOperation::accel_to_orientation(geometry_msgs::PointPtr accel){
    double yaw = fixed_mast_yaw + M_PI; //we want to face the mast
    double roll = atan2(accel->x,9.81);
    double pitch = atan2(accel->y,9.81);
    return euler_to_quaternion(yaw, pitch, roll);
}

void ExtractModuleOperation::update_attitude_input(mavros_msgs::PositionTarget module,mavros_msgs::PositionTarget offset, bool use_sqrt){
    mavros_msgs::PositionTarget ref = addState(module, offset);

    attitude_setpoint.header.stamp = ros::Time::now();
    attitude_setpoint.thrust = 0.5; //this is the thrust that allow a contanst altitude no matter what

    geometry_msgs::PointPtr accel_targ;
    LQR_to_acceleration(&ref, accel_targ, use_sqrt);
    attitude_setpoint.orientation = accel_to_orientation(accel_targ);
}

void ExtractModuleOperation::tick() {
    // Wait until we get the first module position readings before we do anything else.
    if (module_state.header.seq == 0) {
        return;
    }
    
    const double dx = module_state.position.y - getCurrentPose().pose.position.x;
    const double dy = module_state.position.x - getCurrentPose().pose.position.y;

    setpoint.yaw = std::atan2(dy, dx) - M_PI / 18.0;

    const double distance_to_module = sqrt(dx * dx + dy * dy);

    const double dvx = getCurrentTwist().twist.linear.x;
    const double dvy = getCurrentTwist().twist.linear.y;
    const double dvz = getCurrentTwist().twist.linear.z;

    const double drone_speed = sqrt(dvx * dvx + dvy * dvy + dvz * dvz); //not used 

    switch (extraction_state) {
        case ExtractionState::APPROACHING: {
            // TODO: This has to be fixed, should be facing towards the module from any given position,
            // not just from the x direction
            mavros_msgs::PositionTarget smooth_rotated_offset = rotate(transition_state.state,fixed_mast_yaw);

            update_attitude_input(module_state,smooth_rotated_offset, USE_SQRT);
            //TODO: remove this big print
            ROS_INFO_STREAM(ros::this_node::getName().c_str()
                            << ": "
                            << "Approaching, "
                            << "Curent pose : "
                            << std::fixed << std::setprecision(3) //only 3 decimals
                            << getCurrentPose().pose.position.x << " ; "
                            << getCurrentPose().pose.position.y << " ; "
                            << getCurrentPose().pose.position.z
                            << "\tcalculated velocity"
                            << module_state.velocity.x << " ; "
                            << module_state.velocity.y << " ; "
                            << module_state.velocity.z);
            //saveLog();
            //for testing purposes, I toggle the possibility to go to the next step
            //*
            if (distance_to_module < 0.04) {
                extraction_state = ExtractionState::OVER;
                ROS_INFO_STREAM(ros::this_node::getName().c_str()
                                << ": "
                                << "Approaching -> Over");
            }
            //*/
            break;
        }
        case ExtractionState::OVER: {
            setpoint.position.x = module_state.position.y;
            setpoint.position.y = module_state.position.x + 0.78;
            setpoint.position.z = module_state.position.z + 0.3;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.1 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                extraction_state = ExtractionState::BEHIND_WITH_HOOKS;
            }
            
            break;
        }
        case ExtractionState::BEHIND_WITH_HOOKS: {
            setpoint.position.x = module_state.position.y;
            setpoint.position.y = module_state.position.x + 0.78;
            setpoint.position.z = module_state.position.z - 0.1;

            const double distance_to_setpoint =
                Util::distanceBetween(setpoint.position, getCurrentPose().pose.position);

            if (distance_to_setpoint < 0.05 && getCurrentTwist().twist.linear.z < 0.03 && std::abs(getCurrentYaw() - setpoint.yaw) < M_PI / 50.0) {
                extraction_state = ExtractionState::EXTRACTING;
            }

            break;
        }
        case ExtractionState::EXTRACTING: {
            setpoint.position.x = module_state.position.y;
            setpoint.position.y = module_state.position.x + 2.0;
            setpoint.position.z = module_state.position.z - 0.1;

            if (!called_backpropeller_service) {
                std_srvs::SetBool request;
                request.request.data = true;
                backpropeller_client.call(request);
                called_backpropeller_service = true;
            }           

            // If the module is on the way down
            // TODO: this should be checked in a better way
            if (module_state.position.z < 0.5) {
                extraction_state = ExtractionState::EXTRACTED;
                std_srvs::SetBool request;
                request.request.data = false;
                backpropeller_client.call(request);
                called_backpropeller_service = false;
            }

            break;
        }
    }
}
