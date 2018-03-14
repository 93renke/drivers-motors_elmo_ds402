#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::positionToUser(long position) const
{
    return static_cast<double>(position) *
        feedConstant / (positionEncoderResolution * gearRatio);
}

double Factors::velocityToUser(long velocity) const
{
    return static_cast<double>(velocity) *
        feedConstant / (velocityEncoderResolution * gearRatio) *
        velocityFactor;
}

double Factors::accelerationToUser(long acceleration) const
{
    return acceleration * accelerationFactor;
}

double Factors::torqueToUser(long torque) const
{
    return static_cast<double>(torque) / 1000 * ratedTorque;
}

double Factors::currentToUser(long current) const
{
    return static_cast<double>(current) / 1000 * ratedCurrent;
}