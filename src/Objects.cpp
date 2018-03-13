#include <motors_elmo_ds402/Objects.hpp>

using namespace motors_elmo_ds402;

uint16_t ControlWord::encode()
{
    uint16_t word = 0;
    switch(transition)
    {
        case SHUTDOWN:
            word = 0x06;
            break;
        case SWITCH_ON:
            word = 0x07;
            break;
        case ENABLE_OPERATION:
            word = 0x0F;
            break;
        case DISABLE_VOLTAGE:
            word = 0x00;
            break;
        case QUICK_STOP:
            word = 0x02;
            break;
        case DISABLE_OPERATION:
            word = 0x07;
            break;
        case FAULT_RESET:
            word = 0x10;
            break;
    }

    if (enable_halt)
        word |= 0x100;

    return word;
}

StatusWord::State parseState(uint8_t byte)
{
    switch(byte & 0x4F)
    {
        case 0x00: return StatusWord::NOT_READY_TO_SWITCH_ON;
        case 0x40: return StatusWord::READY_TO_SWITCH_ON;
        case 0x0F: return StatusWord::FAULT_REACTION_ACTIVE;
        case 0x08: return StatusWord::FAULT;
    }

    switch(byte & 0x6F)
    {
        case 0x21: return StatusWord::READY_TO_SWITCH_ON;
        case 0x23: return StatusWord::SWITCH_ON;
        case 0x27: return StatusWord::OPERATION_ENABLED;
        case 0x07: return StatusWord::QUICK_STOP_ACTIVE;
    }

    throw StatusWord::UnknownState("received an unknown value for the state");
}

StatusWord StatusWord::parse(uint16_t word)
{
    State state = parseState(word & 0x6F);
    bool voltageEnabled = (word & 0x10);
    bool warning = (word & 0x40);
    bool targetReached = word & 0x400;
    bool internalLimitActive = word & 0x800;
    return StatusWord { state, voltageEnabled, warning,
        targetReached, internalLimitActive };
}