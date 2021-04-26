/**
 * @file data_file.cpp
 */
#include "data_file.h"

DataFile::DataFile(std::string name, std::string path){
    m_name = name;
    if(path == "")
        m_path = std::string(std::getenv("HOME")) + "/"; //store it in home folder by default.
    else
        m_path = path;
    m_precision = 3;
}

void DataFile::init(std::string title){
    //create a header for the data file.
    std::ofstream save_file_f;
    save_file_f.open(m_path+m_name);
    if(save_file_f.is_open())
    {
        save_file_f << title << "\n";
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << m_path+m_name);
    }
}

void DataFile::initStateLog(){
    //create a header for the logfile.
    std::ofstream save_file_f;
    save_file_f.open(m_path+m_name);
    if(save_file_f.is_open())
    {
        save_file_f << "Time\tpose.x\tpose.y\tpose.z"
                    << "\tVel.x\tVel.y\tVel.z"
                    << "\tAccel.x\tAccel.y\tAccel.z"
                    << "\n";
        save_file_f.close();
    }
    else
    {
        ROS_INFO_STREAM(ros::this_node::getName().c_str() << "could not open " << m_path+m_name);
    }
}

void DataFile::saveVector3(const geometry_msgs::Vector3 vec){
    std::ofstream save_file_f;
    save_file_f.open (m_path+m_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(m_precision) //only 3 decimals
                        << ros::Time::now() << "\t"
                        << vec.x << "\t"
                        << vec.y << "\t"
                        << vec.z << "\n";
        save_file_f.close();
    }
}

void DataFile::saveStateLog(const geometry_msgs::Point pose, const geometry_msgs::Vector3 vel, const geometry_msgs::Vector3 accel)
{
    std::ofstream save_file_f;
    save_file_f.open (m_path+m_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(m_precision) //only 3 decimals
                        << ros::Time::now() << "\t"
                        << pose.x << "\t"
                        << pose.y << "\t"
                        << pose.z << "\t"
                        << vel.x << "\t"
                        << vel.y << "\t"
                        << vel.z << "\t"
                        << accel.x << "\t"
                        << accel.y 
                        << "\t" << accel.z
                        << "\n";
        save_file_f.close();
    }
}

void DataFile::saveStateLog(const mavros_msgs::PositionTarget data) //saveLog
{
    saveStateLog(data.position, data.velocity, data.acceleration_or_force);
}

void DataFile::saveArray(double* vec, int n){
    std::ofstream save_file_f;
    save_file_f.open (m_path+m_name, std::ios::app);
    if(save_file_f.is_open())
    {
        save_file_f << std::fixed << std::setprecision(m_precision) //only 3 decimals
                    << ros::Time::now();
        for(int i = 0; i< n ; i++){
            save_file_f << "\t" << vec[i];
        }
        save_file_f << "\n";
        save_file_f.close();
    }
}

void DataFile::setPrecision(int precision){
    m_precision = precision;
}
