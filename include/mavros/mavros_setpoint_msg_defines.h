//
// Created by simengangstad on 07.11.18.
// Source: Control FSM, thanks Håvard <3
// http://discuss.px4.io/t/offboard-takeoff-land-loiter-via-set-position-target-local-ned/2648/2
//

#ifndef FLUID_FSM_MAVROS_SETPOINT_MSG_DEFINES_H
#define FLUID_FSM_MAVROS_SETPOINT_MSG_DEFINES_H

#include <cstdint>

#define IGNORE_PX (1 << 0)    // Position ignore flags
#define IGNORE_PY (1 << 1)
#define IGNORE_PZ (1 << 2)
#define IGNORE_VX (1 << 3)    // Velocity vector ignore flags
#define IGNORE_VY (1 << 4)
#define IGNORE_VZ (1 << 5)
#define IGNORE_AFX (1 << 6)    // Acceleration/Force vector ignore flags
#define IGNORE_AFY (1 << 7)
#define IGNORE_AFZ (1 << 8)
#define FORCE (1 << 9)          // Force in af vector flag
#define IGNORE_YAW (1 << 10)
#define IGNORE_YAW_RATE (1 << 11)
#define SETPOINT_TYPE_TAKEOFF 0x1000
#define SETPOINT_TYPE_LAND 0b0010111111000111 //0x2000
#define SETPOINT_TYPE_LOITER 0x3000
#define SETPOINT_TYPE_IDLE 0x4000
#define MASK_VELOCITY 0b0000111111000111
#define MASK_LAND_VELOCITY 0b0010111111000111

namespace fluid {
    constexpr uint16_t DEFAULT_MASK = IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                                      IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                                      IGNORE_YAW_RATE;

}


#endif //FLUID_FSM_MAVROS_SETPOINT_MSG_DEFINES_H
