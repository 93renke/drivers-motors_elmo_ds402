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
        queryObject<PositionEncoderResolutionNum>(),
        queryObject<PositionEncoderResolutionDen>(),
        queryObject<VelocityEncoderResolutionNum>(),
        queryObject<VelocityEncoderResolutionDen>(),
        queryObject<GearRatioNum>(),
        queryObject<GearRatioDen>(),
        queryObject<FeedConstantNum>(),
        queryObject<FeedConstantDen>(),
        queryObject<VelocityFactorNum>(),
        queryObject<VelocityFactorDen>(),
        queryObject<AccelerationFactorNum>(),
        queryObject<AccelerationFactorDen>()
    };
}

Factors Controller::getFactors() const
{
    double positionEncoderResolution = getRational<
        PositionEncoderResolutionNum,
        PositionEncoderResolutionDen>();

    double velocityEncoderResolution = getRational<
        VelocityEncoderResolutionNum,
        VelocityEncoderResolutionDen>();

    double gearRatio = getRational<
        GearRatioNum,
        GearRatioDen>();

    double feedConstant = getRational<
        FeedConstantNum,
        FeedConstantDen>();

    double velocityFactor = getRational<
        VelocityFactorNum,
        VelocityFactorDen>();

    double accelerationFactor = getRational<
        AccelerationFactorNum,
        AccelerationFactorDen>();

    return Factors {
        positionEncoderResolution,
        velocityEncoderResolution,
        gearRatio,
        feedConstant,
        velocityFactor,
        accelerationFactor
    };
}

#define UPDATE_CASE(object) \
    case (static_cast<uint32_t>(object::OBJECT_ID) << 16 | object::OBJECT_SUB_ID): \
        update |= object::UPDATE_ID; \
        break;

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
        uint32_t fullId = static_cast<uint32_t>(it->first) << 16 | it->second;
        switch(fullId)
        {
            UPDATE_CASE(StatusWord);

            // UPDATE_FACTORS
            UPDATE_CASE(PositionEncoderResolutionNum);
            UPDATE_CASE(PositionEncoderResolutionDen);
            UPDATE_CASE(VelocityEncoderResolutionNum);
            UPDATE_CASE(VelocityEncoderResolutionDen);
            UPDATE_CASE(GearRatioNum);
            UPDATE_CASE(GearRatioDen);
            UPDATE_CASE(FeedConstantNum);
            UPDATE_CASE(FeedConstantDen);
            UPDATE_CASE(VelocityFactorNum);
            UPDATE_CASE(VelocityFactorDen);
            UPDATE_CASE(AccelerationFactorNum);
            UPDATE_CASE(AccelerationFactorDen);
        }
    }

    return Update(update);
}

StatusWord Controller::getStatusWord() const
{
    return get<StatusWord>();
}

template<typename T>
typename T::OBJECT_TYPE Controller::getRaw() const
{
    return mCanOpen.get<typename T::OBJECT_TYPE>(T::OBJECT_ID, T::OBJECT_SUB_ID);
}

template<typename T>
T Controller::get() const
{
    return parse<T, typename T::OBJECT_TYPE>(getRaw<T>());
}

template<typename Num, typename Den>
double Controller::getRational() const
{
    auto num = getRaw<Num>();
    auto den = getRaw<Den>();
    return static_cast<double>(num) / den;
}

template<typename T>
canbus::Message Controller::queryObject() const
{
    return mCanOpen.upload(T::OBJECT_ID, T::OBJECT_SUB_ID);
}

base::JointState Controller::getJointState() const
{

}