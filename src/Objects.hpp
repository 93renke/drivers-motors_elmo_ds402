#ifndef MOTORS_ELMO_DS402_OBJECTS
#define MOTORS_ELMO_DS402_OBJECTS

#include <cstdint>
#include <stdexcept>

namespace motors_elmo_ds402
{
    /** Representation of the heartbeat (NMT state)
     */
    struct Heartbeat
    {
        static const uint64_t UPDATE_ID = 0x00000001;
    };
    /** Representation of the control word
     * 
     * The control word changes the drive's operational state
     */
    struct ControlWord
    {
        enum Transition
        {
            SHUTDOWN,
            SWITCH_ON,
            ENABLE_OPERATION,
            DISABLE_VOLTAGE,
            QUICK_STOP,
            DISABLE_OPERATION,
            FAULT_RESET
        };

        static const int OBJECT_ID = 0x6040;
        static const int OBJECT_SUB_ID = 0x00;
        typedef uint16_t OBJECT_TYPE;

        Transition transition;
        bool enable_halt;

        uint16_t encode();
    };

    /** Representation of the status word
     * 
     * The status word is the main representation of the drive's state
     */
    struct StatusWord
    {
        enum State
        {
            NOT_READY_TO_SWITCH_ON,
            SWITCH_ON_DISABLED,
            READY_TO_SWITCH_ON,
            SWITCH_ON,
            OPERATION_ENABLED,
            QUICK_STOP_ACTIVE,
            FAULT_REACTION_ACTIVE,
            FAULT
        };

        struct UnknownState : public std::runtime_error
        {
            using std::runtime_error::runtime_error;
        };

        static const int OBJECT_ID = 0x6041;
        static const int OBJECT_SUB_ID = 0x00;
        typedef uint16_t OBJECT_TYPE;
        static const uint64_t UPDATE_ID = 0x00000002;

        State state;
        bool voltageEnabled;
        bool warning;
        bool targetReached;
        bool internalLimitActive;

        static StatusWord parse(std::uint16_t word);
    };
}

#endif