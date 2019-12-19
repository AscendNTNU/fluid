
#ifndef FLUID_FSM_TYPE_MASK_H
#define FLUID_FSM_TYPE_MASK_H

#include <cstdint>
namespace fluid {

	namespace TypeMask {

		constexpr uint16_t IgnorePX   = (1 << 0);    	
		constexpr uint16_t IgnorePY   = (1 << 1);
		constexpr uint16_t IgnorePZ   = (1 << 2);
		constexpr uint16_t IgnoreVX   = (1 << 3);    	
		constexpr uint16_t IgnoreVY   = (1 << 4);
		constexpr uint16_t IgnoreVZ   = (1 << 5);
		constexpr uint16_t IgnoreAFX  = (1 << 6);
		constexpr uint16_t IgnoreAFY  =	(1 << 7);
		constexpr uint16_t IgnoreAFZ  = (1 << 8);
		constexpr uint16_t Force      = (1 << 9); 
		constexpr uint16_t IgnoreYaw  = (1 << 10);
		constexpr uint16_t IgnoreYawRate = (1 << 11);
		constexpr uint16_t Idle = 0x4000;

	    constexpr uint16_t Position = IgnoreVX | IgnoreVY | IgnoreVZ |
                                      IgnoreAFX | IgnoreAFY | IgnoreAFZ |
                              	 	  IgnoreYawRate;
										 
 	    constexpr uint16_t Velocity = IgnorePX | IgnorePY | IgnorePZ |
                                      IgnoreAFX | IgnoreAFY | IgnoreAFZ |
                              	 	  IgnoreYawRate;
	}
}


#endif
