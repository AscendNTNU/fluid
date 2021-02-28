/**
 * @file mast.h
 */

#ifndef MAST_H
#define MAST_H

#include "operation.h" //it has all the includes needed.

//mast movement estimation
#define SAVE_PITCH_TIME 15
#define SAVE_PITCH_FREQ 4 //Todo, may not work with whatever frequency. 6Hz looks weird

/**
 * @brief Represent the mast of Mission 9
 */
class Mast{
   
    private:
    /**
     * @brief Enable debugging prints
     */
    bool m_SHOW_PRINTS;

     /**
     * @brief Should be given by perception and known before entering
     *  InteractOperation
     */
    float m_fixed_yaw;

    /**
     * @brief the pitch, roll and trigonometric angle of the mast
     */
    geometry_msgs::Vector3 m_angle;

    /**
     * @brief The estimated period of the pitch of the mast.
     * -1 if not estimated yet.
     */
    float m_period;
    
    /**
     * @brief Save of the mast pitch over the last 20sec at 5Hz
     */
    float* m_pitches;

    /**
     * @brief The last index a mast pitch has been saved in
     */
    uint16_t m_pitches_id;

    /**
     * @brief keep track of when is the pitch is saved for mast period estimation
     * 
     */
    ros::Time m_last_time_pitch_saved;

    /**
     * @brief Define if the mast pitch is increasing or decreasing
     * 
     */
    bool m_lookForMin;

    /**
     * @brief current extremum value being tested to replace the last one.
     * 
     */
    float m_current_extremum;
    
    /**
     * @brief The minimum pitch the mast got during the last oscillation
     * 
     */
    float m_last_min_pitch;

    /**
     * @brief The maximum pitch the mast got during the last oscillation
     * 
     */
    float m_last_max_pitch;

    /**
     * @brief time at whitch the last minimum pitch has been found
     * 
     */
    ros::Time m_time_last_min_pitch;
    ros::Time m_time_last_max_pitch;

    /**
     * @brief Look for the indice of the minimum value in arraw 
     * within the given boudaries
     * 
     * @param array Is the arraw in which the minimum is looked for
     * @param begin The beginning of the range of the research
     * @param end The end of the range of the research
     * @return The indice of the minimum value
     */
    int search_min_id_within(float* array, int begin, int end);

    /**
     * @brief Look for the indice of the maximum value in arraw 
     * within the given boudaries
     * 
     * @param array Is the arraw in which the maximum is looked for
     * @param begin The beginning of the range of the research
     * @param end The end of the range of the research
     * @return The indice of the maximum value
     */
    int search_max_id_within(float* array, int begin, int end);

    /**
     * @brief save the actual mast pitch into an arraw.
     * It will later be used to estimate future orientation
     */
    void save_pitch();

    /**
     * @brief Estimate the mast pitch oscillation period from
     * the measurment saved with save_pitch
     */
    void estimate_period();

    
    
    
    public:
    /**
     * @brief Construct a new Mast object
     * 
     * @param yaw The fixed yaw angle of mast. 
     * Should be calculated by perception and given by AI
     */
    Mast(float yaw=0.0);

    /**
     * @brief Update mast euler angles
     * 
     * @param orientation quaternion mast or module orientation
     */
    void update(geometry_msgs::Quaternion orientation); //todo: this should also save the pitch automaticaly

    /**
     * @brief Update mast euler angles.
     *        Check if pitch were extremum.
     *        Deduce period.
     * 
     * @param orientation quaternion mast or module orientation
     */
    void update2(geometry_msgs::Quaternion orientation); //todo: this should also save the pitch automaticaly

    /**
     * @brief Estimate the time the mast will take to reach its next 
     *        most forward position = maximum pitch.
     *        It considers that the current oscillation is exactly the 
     *        same as the previous one
     * 
     * @return Time until the mast reaches its next maximum pitch
     */
    float time_to_max_pitch();
    
    /**
     * @brief Get the mast fixed yaw 
     * 
     * @return float mast fixed yaw
     */
    float get_yaw();

    /**
     * @brief Get the estimated period of the mast
     * 
     * @return -1 if not estimated yet, the estimated period otherwise
     */
    float get_period();

};
#endif // MAST_H