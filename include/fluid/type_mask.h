
#ifndef TYPE_MASK_H
#define TYPE_MASK_H

#include <cstdint>

class TypeMask {
public:
    static constexpr uint16_t IgnorePX = (1 << 0);
    static constexpr uint16_t IgnorePY = (1 << 1);
    static constexpr uint16_t IgnorePZ = (1 << 2);
    static constexpr uint16_t IgnoreVX = (1 << 3);
    static constexpr uint16_t IgnoreVY = (1 << 4);
    static constexpr uint16_t IgnoreVZ = (1 << 5);
    static constexpr uint16_t IgnoreAFX = (1 << 6);
    static constexpr uint16_t IgnoreAFY = (1 << 7);
    static constexpr uint16_t IgnoreAFZ = (1 << 8);
    static constexpr uint16_t Force = (1 << 9);
    static constexpr uint16_t IgnoreYaw = (1 << 10);
    static constexpr uint16_t IgnoreYawRate = (1 << 11);
    static constexpr uint16_t Idle = 0x4000;

    static constexpr uint16_t Position =
        IgnoreVX | IgnoreVY | IgnoreVZ | IgnoreAFX | IgnoreAFY | IgnoreAFZ | IgnoreYawRate;

    static constexpr uint16_t Velocity =
        IgnorePX | IgnorePY | IgnorePZ | IgnoreAFX | IgnoreAFY | IgnoreAFZ | IgnoreYawRate;
};

#endif
