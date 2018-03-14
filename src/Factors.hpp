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
        double accelerationFactor        = base::unknown<double>();
        double ratedTorque               = base::unknown<double>();
        double ratedCurrent              = base::unknown<double>();

        double positionToUser(long encoder) const;
        double velocityToUser(long encoder) const;
        double accelerationToUser(long acc) const;
        double torqueToUser(long torque) const;
        double currentToUser(long current) const;
    };
}

#endif
