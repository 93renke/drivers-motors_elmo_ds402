#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::positionToUser(int32_t position) const
{
    return static_cast<double>(position) *
        feedConstant / (positionEncoderResolution * gearRatio);
}

double Factors::velocityToUser(int32_t velocity) const
{
    return static_cast<double>(velocity) *
        feedConstant / (velocityEncoderResolution * gearRatio) *
        velocityFactor;
}

double Factors::torqueToUser(int16_t torque) const
{
    return static_cast<double>(torque) / 1000 * ratedTorque;
}

double Factors::currentToUser(int16_t current) const
{
    return static_cast<double>(current) / 1000 * ratedCurrent;
}