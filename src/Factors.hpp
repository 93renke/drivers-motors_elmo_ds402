#ifndef MOTORS_ELMO_DS402_FACTORS_HPP
#define MOTORS_ELMO_DS402_FACTORS_HPP

#include <cstdint>
#include <base/Float.hpp>

namespace motors_elmo_ds402
{
    struct Factors
    {
        int64_t encoderTicks = 1;
        int64_t encoderRevolutions = 1;
        int64_t gearMotorShaftRevolutions   = 1;
        int64_t gearDrivingShaftRevolutions = 1;
        int64_t feedLength = 1;
        int64_t feedDrivingShaftRevolutions = 1;
        double ratedCurrent = base::unknown<double>();
        double ratedTorque  = base::unknown<double>();

        void update();
        double scaleEncoderValue(int64_t encoder) const;
        double currentToUserTorque(int64_t current) const;
        double currentToUser(int64_t current) const;

        int64_t positionNumerator = 1;
        int64_t positionDenominator = 1;
    };
}

#endif
