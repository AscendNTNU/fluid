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
     * @brief Save of the mast pitch over the last 20sec at 5Hz
     */
    float* m_pitches;

    /**
     * @brief The last index a mast pitch has been saved in
     */
    uint16_t m_pitches_id;

    
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


    public:
    /**
     * @brief Construct a new Mast object
     * 
     * @param yaw The fixed yaw angle of mast. 
     * Should be calculated by perception and given by AI
     */
    Mast(float yaw=0.0);

    /**
     * @brief save the actual mast pitch into an arraw.
     * It will later be used to estimate future orientation
     * 
     * @param save_rate The rate at which it is being saved
     */
    void save_pitch(int save_rate);

    /**
     * @brief Estimate the mast pitch oscillation period from
     * the measurment saved with save_pitch
     * 
     * @param save_rate 
     */
    void estimate_period(int save_rate);

    /**
     * @brief Update mast euler angles
     * 
     * @param orientation quaternion mast or module orientation
     */
    void update(geometry_msgs::Quaternion orientation); //todo: this should also save the pitch automaticaly

    
    /**
     * @brief Get the mast fixed yaw 
     * 
     * @return float mast fixed yaw
     */
    float get_yaw();

};
#endif // MAST_H