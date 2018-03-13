#include <motors_elmo_ds402/Factors.hpp>

using namespace std;
using namespace motors_elmo_ds402;

double Factors::positionToUser(int32_t position)
{
    return static_cast<double>(position) *
        feedConstant / (positionEncoderResolution * gearRatio);
}

double Factors::velocityToUser(int32_t velocity)
{
    return static_cast<double>(velocity) *
        feedConstant / (velocityEncoderResolution * gearRatio) *
        velocityFactor;
}