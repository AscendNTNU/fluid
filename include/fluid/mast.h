/**
 * @file mast.h 
 */


#ifndef MAST_H
#define MAST_H

#include "operation.h" //it has all the includes needed and is already included anyway
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
     * @brief state of the interaction_point. Includes Position, velocity and acceleration and a header.
     * 
     */
    mavros_msgs::PositionTarget interaction_point_state;

    /**
     * @brief state of the interaction_point at the previous iteration/tick.
     *  Includes Position, velocity and acceleration and a header.
     * 
     */
    mavros_msgs::PositionTarget previous_interaction_point_state;


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

    /**
     * @brief time at whitch the last maximum pitch has been found
     * 
     */
    ros::Time m_time_last_max_pitch;

    
    
    
    public:
    /**
     * @brief Construct a new Mast object
     * 
     * @param yaw The fixed yaw angle of mast. 
     * Should be calculated by perception and given by AI
     */
    Mast(float yaw=0.0);

    /**
     * @brief Update position and velocity from EKF output
     * 
     * @param module_state state of the interaction_point from the EKF
     */
    void updateFromEkf(mavros_msgs::PositionTarget module_state);

    /**
     * @brief Update position, velocity and acceleration from euler derivations
     * 
     * @param module_pose_ptr state of the interaction_point
     */
    void update(geometry_msgs::PoseStamped module_pose_ptr); //todo: this should also save the pitch automaticaly

    /**
     * @brief Check if pitch were extremum.
     *        Deduce period.
     * 
     * @param pitch from the current orientation of the mast. Can be noisy.
     */
    void search_period(double pitch);

    /**
     * @brief estimate the velocity of the interaction point from a simple Euler derivation of the position
     * 
     */
    void estimateInteractionPointVel();

    /**
     * @brief estimate the acceleration of the interaction point from a simple Euler derivation of the velocity
     * 
     */
    void estimateInteractionPointAccel();

    /**
     * @brief Estimate the time the mast will take to reach its next 
     *        most forward position = maximum pitch.
     *        It considers that the current oscillation is exactly the 
     *        same as the previous one and that the pitch is a triangular function
     * 
     * @return -1 if not defined. Else, time until the mast reaches its next maximum pitch
     */
    float time_to_max_pitch();
    
    /**
     * @brief Estimate the time the mast will take to reach its next 
     *        most forward position = maximum pitch, using arcsin.
     *        It considers that the current oscillation is exactly the 
     *        same as the previous one.
     * 
     * @return -1 if not defined. Else, time until the mast reaches its next maximum pitch
     */
    float time_to_max_pitch_accurate();

    /**
     * @brief Get the mast fixed yaw 
     * 
     * @return float mast fixed yaw
     */
    float get_yaw();

    /**
     * @brief Set the period of the mast
     * 
     * @param period from ekf
     */
    void set_period(float period);

    /**
     * @brief Get the estimated period of the mast
     * 
     * @return -1 if not estimated yet, the estimated period otherwise
     */
    float get_period();

    /**
     * @brief Get the interaction point state object
     * 
     * @return mavros_msgs::PositionTarget 
     */
    mavros_msgs::PositionTarget get_interaction_point_state();

};
#endif // MAST_H