//
// Created by simengangstad on 07.11.18.
// Source: Control FSM, thanks Håvard <3
// http://discuss.px4.io/t/offboard-takeoff-land-loiter-via-set-position-target-local-ned/2648/2
//

#ifndef FLUID_FSM_TYPE_MASK_H
#define FLUID_FSM_TYPE_MASK_H

#include <cstdint>

#define IGNORE_PX 		(1 << 0)    	// Position ignore flags
#define IGNORE_PY 		(1 << 1)
#define IGNORE_PZ 		(1 << 2)
#define IGNORE_VX 		(1 << 3)    	// Velocity vector ignore flags
#define IGNORE_VY 		(1 << 4)
#define IGNORE_VZ  		(1 << 5)
#define IGNORE_AFX 		(1 << 6)    	// Acceleration/Force vector ignore flags
#define IGNORE_AFY 		(1 << 7)
#define IGNORE_AFZ 		(1 << 8)
#define FORCE 	   		(1 << 9)     	// Force in af vector flag
#define IGNORE_YAW 		(1 << 10)
#define IGNORE_YAW_RATE (1 << 11)

namespace fluid {

	namespace TypeMask {

	    constexpr uint16_t Default = IGNORE_VX | IGNORE_VY | IGNORE_VZ |
                                     IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ |
                              	 	 IGNORE_YAW_RATE;
              	 	 
		constexpr uint16_t Idle    = IGNORE_PX | IGNORE_PY | IGNORE_PZ;
	}
}


#endif
