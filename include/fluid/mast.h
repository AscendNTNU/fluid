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
    Mast(float yaw=0.0);
    void save_mast_pitch(int save_rate);
    void estimate_mast_period(int save_rate);
    void update(geometry_msgs::Quaternion orientation);

    float get_yaw();

};
#endif // MAST_H