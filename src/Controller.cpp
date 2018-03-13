#include <motors_elmo_ds402/Controller.hpp>

using namespace motors_elmo_ds402;


Controller::Controller(uint8_t nodeId)
    : mCanOpen(nodeId)
{
}

canbus::Message Controller::queryNodeState() const
{
    return mCanOpen.queryState();
}

canbus::Message Controller::queryNodeStateTransition(
            canopen_master::NODE_STATE_TRANSITION transition) const
{
    return mCanOpen.queryStateTransition(transition);
}

canopen_master::NODE_STATE Controller::getNodeState() const
{
    return mCanOpen.getState();
}

canbus::Message Controller::queryStatusWord() const
{
    return mCanOpen.upload(StatusWord::OBJECT_ID, 0);
}

std::vector<canbus::Message> Controller::queryFactors()
{
    return std::vector<canbus::Message>
    {
        // Position encoder
        mCanOpen.upload(0x608F, 1),
        mCanOpen.upload(0x608F, 2),
        // Velocity encoder
        mCanOpen.upload(0x6090, 1),
        mCanOpen.upload(0x6090, 2),
        // Gear Ratio
        mCanOpen.upload(0x6091, 1),
        mCanOpen.upload(0x6091, 2),
        // Feed constant
        mCanOpen.upload(0x6092, 1),
        mCanOpen.upload(0x6092, 2),
        // Velocity Factor
        mCanOpen.upload(0x6096, 1),
        mCanOpen.upload(0x6096, 2),
        // Acceleration Factor
        mCanOpen.upload(0x6097, 1),
        mCanOpen.upload(0x6097, 2)
    };
}

Factors Controller::getFactors() const
{
    // Position encoder
    uint32_t positionEncoderNum = mCanOpen.get<uint32_t>(0x608F, 1);
    uint32_t positionEncoderDen = mCanOpen.get<uint32_t>(0x608F, 2);
    // Velocity encoder
    uint32_t velocityEncoderNum = mCanOpen.get<uint32_t>(0x6090, 1);
    uint32_t velocityEncoderDen = mCanOpen.get<uint32_t>(0x6090, 2);
    // Gear Ratio
    uint32_t gearRatioNum = mCanOpen.get<uint32_t>(0x6091, 1);
    uint32_t gearRatioDen = mCanOpen.get<uint32_t>(0x6091, 2);
    // Feed constant
    uint32_t feedConstantNum = mCanOpen.get<uint32_t>(0x6092, 1);
    uint32_t feedConstantDen = mCanOpen.get<uint32_t>(0x6092, 2);
    // Velocity Factor
    uint32_t velocityFactorNum = mCanOpen.get<uint32_t>(0x6096, 1);
    uint32_t velocityFactorDen = mCanOpen.get<uint32_t>(0x6096, 2);
    // Acceleration Factor
    uint32_t accelerationFactorNum = mCanOpen.get<uint32_t>(0x6097, 1);
    uint32_t accelerationFactorDen = mCanOpen.get<uint32_t>(0x6097, 2);

    return Factors {
        static_cast<double>(positionEncoderNum) / positionEncoderDen,
        static_cast<double>(velocityEncoderNum) / velocityEncoderDen,
        static_cast<double>(gearRatioNum) / gearRatioDen,
        static_cast<double>(feedConstantNum) / feedConstantDen,
        static_cast<double>(velocityFactorNum) / velocityFactorDen,
        static_cast<double>(accelerationFactorNum) / accelerationFactorDen,
    };
}

Update Controller::process(canbus::Message const& msg)
{
    uint64_t update = 0;
    auto canUpdate = mCanOpen.process(msg);
    if (canUpdate.mode == canopen_master::StateMachine::PROCESSED_HEARTBEAT)
    {
        update |= Heartbeat::UPDATE_ID;
    }
    for (auto it = canUpdate.begin(); it != canUpdate.end(); ++it)
    {
        switch(it->first)
        {
            case StatusWord::OBJECT_ID:
                update |= StatusWord::UPDATE_ID;
                break;
            case 0x608F:
            case 0x6090:
            case 0x6091:
            case 0x6092:
            case 0x6096:
            case 0x6097:
                update |= UPDATE_FACTORS;
                break;
        }
    }

    return Update(update);
}

StatusWord Controller::getStatusWord() const
{
    return get<StatusWord>();
}

template<typename T>
T Controller::get() const
{
    return T::parse(mCanOpen.get<typename T::OBJECT_TYPE>(T::OBJECT_ID, T::OBJECT_SUB_ID));
}
