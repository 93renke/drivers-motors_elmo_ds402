#ifndef MOTORS_ELMO_DS402_FACTORS_HPP
#define MOTORS_ELMO_DS402_FACTORS_HPP

#include <cstdint>
#include <base/Float.hpp>

namespace motors_elmo_ds402
{
    struct Factors
    {
        double positionEncoderResolution = base::unknown<double>();
        double velocityEncoderResolution = base::unknown<double>();
        double gearRatio                 = base::unknown<double>();
        double feedConstant              = base::unknown<double>();
        double velocityFactor            = base::unknown<double>();
        double ratedTorque               = base::unknown<double>();
        double ratedCurrent              = base::unknown<double>();

        double positionToUser(std::int32_t encoder) const;
        double velocityToUser(std::int32_t encoder) const;
        double torqueToUser(std::int16_t torque) const;
        double currentToUser(std::int16_t current) const;
    };
}

#endif
