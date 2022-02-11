/**
 * @file data_file.h
 */

#ifndef DATA_FILE_H
#define DATA_FILE_H

#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>


/**
 * @brief Holds a bunch of convenience functions to save data into txt files
 */
class DataFile {
    private:
    
    int m_precision;
    bool m_save_z;
    std::string m_path;
    std::string m_name;

    
    public:
    DataFile(std::string name="noname.txt",std::string path = "");

    /**
     * @brief initialize the data file with a title. 
     * 
     * @param title array of names separated by tabulations
     */
    void init(std::string title);
    
    void initStateLog();

    void saveVector3(const geometry_msgs::msg::Vector3 vec);

    void saveStateLog(const geometry_msgs::msg::Point pose, const geometry_msgs::msg::Vector3 vel, const geometry_msgs::msg::Vector3 accel); //saveLog
    
    void saveStateLog(const mavros_msgs::msg::PositionTarget data);
    
    void saveArray(double* vec, int n);

    void setPrecision(int precision);

    void shouldSaveZ(bool save_z);
};
#endif