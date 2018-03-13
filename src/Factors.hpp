#ifndef MOTORS_ELMO_DS402_FACTORS_HPP
#define MOTORS_ELMO_DS402_FACTORS_HPP

#include <cstdint>

namespace motors_elmo_ds402
{
    struct Factors
    {
        double positionEncoderResolution;
        double velocityEncoderResolution;
        double gearRatio;
        double feedConstant;
        double velocityFactor;
        double accelerationFactor;

        double positionToUser(std::int32_t encoder);
        double velocityToUser(std::int32_t encoder);
    };
}

#endif
