/**
 * @file mast.cpp
 */
#include "mast.h"
#include "util.h"
#include "fluid.h"

Mast::Mast(float yaw){
    m_fixed_yaw = yaw;
    m_period = 10;
    m_pitches = (float*) calloc(SAVE_PITCH_FREQ*SAVE_PITCH_TIME,sizeof(float));
    m_pitches_id=0;
    m_SHOW_PRINTS = Fluid::getInstance().configuration.interaction_show_prints;
    m_time_last_min_pitch;
    m_time_last_max_pitch;
    m_current_extremum = 0;
}

void Mast::update2(geometry_msgs::Quaternion orientation){
    const geometry_msgs::Vector3 mast_euler_angle = Util::quaternion_to_euler_angle(orientation);
    m_angle.x =  mast_euler_angle.y;
    m_angle.y = -mast_euler_angle.z;
    m_angle.z =  180.0/M_PI * atan2(mast_euler_angle.y,-mast_euler_angle.z);

    if(m_lookForMin)
    {
        if(m_angle.x < m_current_extremum){
            m_current_extremum = m_angle.x;
            m_time_last_min_pitch = ros::Time::now();
        }
        else if ( (ros::Time::now() - m_time_last_max_pitch) >= ros::Duration(m_period/3.0) ){ //todo: may cause some trouble at the init
            if ((ros::Time::now() - m_time_last_min_pitch) >= ros::Duration(0.5)){ //todo: the last extremum, may be the wrong one, this test is not sufficient
                //We have not found a new minimum for 0.5sec. The last one found it the correct one.
                m_last_min_pitch = m_current_extremum;
                m_lookForMin = false;
                if(!m_time_last_max_pitch.isZero()){ //We have already found a min before
                    m_period = 2* (m_time_last_min_pitch - m_time_last_max_pitch).toSec();
                    ROS_INFO_STREAM("period from min = " << m_period);
                }
            }
        }
    }
    else{ //If we do no look for a min, we look for a max
        if (m_angle.x > m_current_extremum){
            m_current_extremum = m_angle.x;
            m_time_last_max_pitch = ros::Time::now();
        }
        else if ( (ros::Time::now() - m_time_last_min_pitch) >= ros::Duration(m_period/3.0) ){ //todo: may cause some trouble at the init
            if ( (ros::Time::now() - m_time_last_max_pitch) >= ros::Duration(0.5)){ //todo: the last extremum, may be the wrong one, this test is not sufficient
                //We have not found a new maximum for 0.5sec. The last one found it the correct one.
                m_last_max_pitch = m_current_extremum;
                m_lookForMin = true;
                if(!m_time_last_min_pitch.isZero()){ //We have already found a min before
                    m_period = 2 * (m_time_last_max_pitch - m_time_last_min_pitch).toSec();
                    ROS_INFO_STREAM("period from max = " << m_period);
                }
            }
        }
    }
}

void Mast::update(geometry_msgs::Quaternion orientation){
    
    const geometry_msgs::Vector3 mast_euler_angle = Util::quaternion_to_euler_angle(orientation);
    m_angle.x =  mast_euler_angle.y;
    m_angle.y = -mast_euler_angle.z;
    m_angle.z =  180.0/M_PI * atan2(mast_euler_angle.y,-mast_euler_angle.z);
    
    if(ros::Time::now()-m_last_time_pitch_saved >= ros::Duration(2.0/(float)SAVE_PITCH_FREQ)){
        // We are late on saving. We are just gonna restart from here.
        m_last_time_pitch_saved = ros::Time::now();
        m_pitches_id = 0;
    }
    else if(ros::Time::now()-m_last_time_pitch_saved >= ros::Duration(1.0/(float)SAVE_PITCH_FREQ)){
        m_last_time_pitch_saved.operator+=(ros::Duration(1.0/SAVE_PITCH_FREQ));
        save_pitch();
    }
}

void Mast::save_pitch(){
    m_pitches[m_pitches_id] = m_angle.x;
    m_pitches_id++;
//    printf("mast angle n°%d: %f\n",m_pitches_id,mast_angle.x);
    if(m_pitches_id==SAVE_PITCH_FREQ*SAVE_PITCH_TIME){
        m_pitches_id=0;
        estimate_period();
    }
}

void Mast::estimate_period(){
    // perception should get it with the kalman filter, but nice to have it ourselves I guess

    // This first version consider that the newest values are the lowest indexes.
    // It also consider that we never get out of bounds

    // We try to find the min and the max looking at successif short intervals
    // By taking 2 seconds interval, we know that we won't have more than a full
    // period, and therefore, we can't have two local extremum.
    // If the indice of the extremum is the beginning indice or the end indice
    // that means that we probably did not find it.
    int min_id=0, max_id=0;
    int interval_end;
    //First, we estimate whether the pitches is increasing or decreasing
    if(m_pitches[0] > m_pitches[5]) {
        // pitches in decreasing, we first look for the min
        do {
            interval_end = min_id+3*SAVE_PITCH_FREQ;
            min_id = search_min_id_within(m_pitches,min_id,interval_end);
        } while (min_id >= interval_end-SAVE_PITCH_FREQ/2-1); // -1 because the last indice is not included in the previous search
        // let us look for the max now
        max_id = min_id + 3*SAVE_PITCH_FREQ; //save some iterations
        do {
            interval_end = max_id+3*SAVE_PITCH_FREQ;
            max_id = search_max_id_within(m_pitches,max_id,interval_end);
        } while (max_id >= interval_end-SAVE_PITCH_FREQ/2-1); // -1 because the last indice is not included in the previous search
                                                    // -SAVE_PITCH_FREQ/2 to take some margin as the signal may be noisy
        
    }
    else {
        // pitches in decreasing, we first look for the min
        do {
            interval_end = max_id+3*SAVE_PITCH_FREQ;
            max_id = search_max_id_within(m_pitches,max_id,interval_end);
        } while (max_id >= interval_end-SAVE_PITCH_FREQ/2-1); // -1 because the last indice is not included in the previous search
        // let us look for the min now
        min_id = max_id + 3*SAVE_PITCH_FREQ; //save some iterations
        do {
            interval_end = min_id+3*SAVE_PITCH_FREQ;
            min_id = search_min_id_within(m_pitches,min_id,interval_end);
        } while (min_id >= interval_end-SAVE_PITCH_FREQ/2-1); // -1 because the last indice is not included in the previous search
    }
    m_period = 2.0 * (float)abs(min_id - max_id)/(float)SAVE_PITCH_FREQ;
    
    printf("The mast period is %f\n\n", m_period);

    
}

int Mast::search_min_id_within(float* array, int begin, int end){
    int min_id = begin;
    for(int i = begin ; i < end ; i++)
    {
        if(array[i] <= array[min_id])
            min_id = i;
    }
    return min_id;
}

int Mast::search_max_id_within(float* array, int begin, int end){
    int max_id = begin;
    for(int i = begin ; i < end ; i++)
    {
        if(array[i] >= array[max_id])
            max_id = i;
    }
    return max_id;
}

float Mast::time_to_max_pitch(){
    //check that we got both a min and a max
    if(!m_time_last_max_pitch.isZero() && !m_time_last_min_pitch.isZero())
    {
        if(m_lookForMin){
            //the mast is pitching backward:
            return m_period/2.0 *(2.0 - (m_last_max_pitch-m_angle.x)/(m_last_max_pitch-m_last_min_pitch));
        }
        else{
            //the mast is pitching frontward:
            return m_period/2.0 *( 1.0 - (m_angle.x- m_last_min_pitch)/(m_last_max_pitch-m_last_min_pitch));
        }
    }
    return -1;
}

//TODO: do a max pitch ETA to see if it seems constant

float Mast::get_yaw(){
    return m_fixed_yaw;
}

float Mast::get_period(){
    return m_period;
}