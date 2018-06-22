#ifndef MOTORS_ELMO_DS402_MOTOR_PARAMETERS_HPP
#define MOTORS_ELMO_DS402_MOTOR_PARAMETERS_HPP

#include <base/Float.hpp>
#include <cstdint>

namespace motors_elmo_ds402 {
    /**
     * User-configurable part of the factors that translate from internal values
     * to SI-values
     */
    struct MotorParameters {
        /**
         * Number of ticks per \c encoderRevolution
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t encoderTicks = 0;
        /**
         * Number of revolutions in which the encoder reports \c encoderTicks
         * ticks
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t encoderRevolutions = 0;
        /**
         * Motor-side of the gear ratio
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t gearMotorShaftRevolutions   = 0;
        /**
         * Driving-side of the gear ratio
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t gearDrivingShaftRevolutions = 0;
        /**
         * Feed length per \c feedDrivingShaftRevolutions for linear actuators
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t feedLength = 0;
        /**
         * Number of revolutions in which the actuator does \c feedLength
         * for linear actuators
         *
         * Leave to zero to keep the drive-provided value
         */
        int64_t feedDrivingShaftRevolutions = 0;
        /**
         * The motor constant in Nm/A
         *
         * It is used to compute the rated torque based on the drive-reported
         * rated current
         */
        double torqueConstant = base::unset<double>();
    };
}

#endif
