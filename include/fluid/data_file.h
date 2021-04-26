/**
 * @file data_file.h
 */

#ifndef DATA_FILE_H
#define DATA_FILE_H

#include <iostream>
#include <fstream>
#include <unistd.h> //to get the current directory

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>


/**
 * @brief Holds a bunch of convenience functions to save data into txt files
 */
class DataFile {
    private:
    
    int m_precision;
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
    
    void initStateLog(); // initLog

    void saveVector3(const geometry_msgs::Vector3 vec);// saveSetpointLog

    void saveStateLog(const geometry_msgs::Point pose, const geometry_msgs::Vector3 vel, const geometry_msgs::Vector3 accel); //saveLog
    
    void saveStateLog(const mavros_msgs::PositionTarget data); //saveLog
    
    void saveArray(double* vec, int n);

    void setPrecision(int precision);
};
#endif