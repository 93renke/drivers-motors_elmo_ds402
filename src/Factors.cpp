#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::scaleEncoderValue(int64_t encoder) const
{
    int64_t turns = encoder * positionNumerator / positionDenominator;
    int64_t remainingEncoder = encoder - turns * positionDenominator;
    double remaining = static_cast<double>(remainingEncoder) * positionNumerator /
        positionDenominator;
    return 2 * M_PI * turns + 2 * M_PI * remaining;
}

double Factors::currentToUserTorque(long current) const
{
    return static_cast<double>(current) / 1000 * ratedTorque;
}

double Factors::currentToUser(long current) const
{
    return static_cast<double>(current) / 1000 * ratedCurrent;
}

void Factors::update()
{
    positionNumerator =
        feedLength *
        encoderRevolutions *
        gearDrivingShaftRevolutions;
    positionDenominator =
        feedDrivingShaftRevolutions *
        encoderTicks *
        gearMotorShaftRevolutions;
}
