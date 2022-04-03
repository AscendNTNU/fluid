#ifndef CONFIG_H
#define CONFIG_H
/* 
This is an incredibly bad solution, but a fast one.
NEVER INCLUDE THIS FILE IN ANY OTHER SOURCE FILE THAN MAIN.CPP
*/

//DEFAULT VALUES
bool ekf =                              false;
bool use_perception =                   false;
int refresh_rate =                      20;
bool should_auto_arm =                  false;
bool should_auto_offboard =             false;
float distance_completion_threshold =   0.30;
float velocity_completion_threshold =   0.10;
float default_height =                  2.0;
bool launch_rviz =                      false;
bool interaction_show_prints =          false;
float interaction_max_vel =             0.30;
float interaction_max_acc =             0.23;
float travel_max_angle =                70;
float travel_speed =                    15;
float travel_accel =                    10;
  
float fh_offset_x =                     0.42;
float fh_offset_y =                     0.02;
float fh_offset_z =                     -0.10;

//SIMULATOR VALUES
bool use_simulator =                    false;
bool sim_ekf =                          false;
bool sim_use_perception =               false;

#endif // CONFIG_H
