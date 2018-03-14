#include <motors_elmo_ds402/Controller.hpp>

using namespace std;
using namespace motors_elmo_ds402;

Controller::Controller(uint8_t nodeId)
    : mCanOpen(nodeId)
    , mRatedTorque(base::unknown<double>())
{
}

void Controller::setRatedTorque(double ratedTorque)
{
    mRatedTorque = ratedTorque;
}

double Controller::getRatedTorque() const
{
    return mRatedTorque;
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
        queryObject<MotorRatedCurrent>()
    };
}

Factors Controller::getFactors() const
{
    return mFactors;
}

Factors Controller::computeFactors() const
{
    Factors factors;

    factors.positionEncoderResolution = getRational<
        PositionEncoderResolutionNum,
        PositionEncoderResolutionDen>();

    factors.velocityEncoderResolution = getRational<
        VelocityEncoderResolutionNum,
        VelocityEncoderResolutionDen>();

    factors.gearRatio = getRational<
        GearRatioNum,
        GearRatioDen>();

    factors.feedConstant = getRational<
        FeedConstantNum,
        FeedConstantDen>();

    factors.velocityFactor = getRational<
        VelocityFactorNum,
        VelocityFactorDen>();

    factors.ratedTorque  = mRatedTorque;
    factors.ratedCurrent = static_cast<double>(getRaw<MotorRatedCurrent>()) / 1000;
    return factors;
}

#define MODE_UPDATE_CASE(mode, object) \
    case canopen_master::StateMachine::mode: \
        update |= object::UPDATE_ID; \
        break;

#define SDO_UPDATE_CASE(object) \
    case (static_cast<uint32_t>(object::OBJECT_ID) << 16 | object::OBJECT_SUB_ID): \
        update |= object::UPDATE_ID; \
        break;

Update Controller::process(canbus::Message const& msg)
{
    uint64_t update = 0;
    auto canUpdate = mCanOpen.process(msg);
    switch(canUpdate.mode)
    {
        MODE_UPDATE_CASE(PROCESSED_HEARTBEAT, Heartbeat);
        default: ; // we just ignore the rest, we really don't care
    };

    for (auto it = canUpdate.begin(); it != canUpdate.end(); ++it)
    {
        uint32_t fullId = static_cast<uint32_t>(it->first) << 16 | it->second;
        switch(fullId)
        {
            SDO_UPDATE_CASE(StatusWord);

            // UPDATE_FACTORS
            SDO_UPDATE_CASE(PositionEncoderResolutionNum);
            SDO_UPDATE_CASE(PositionEncoderResolutionDen);
            SDO_UPDATE_CASE(VelocityEncoderResolutionNum);
            SDO_UPDATE_CASE(VelocityEncoderResolutionDen);
            SDO_UPDATE_CASE(GearRatioNum);
            SDO_UPDATE_CASE(GearRatioDen);
            SDO_UPDATE_CASE(FeedConstantNum);
            SDO_UPDATE_CASE(FeedConstantDen);
            SDO_UPDATE_CASE(VelocityFactorNum);
            SDO_UPDATE_CASE(VelocityFactorDen);
            SDO_UPDATE_CASE(MotorRatedCurrent);

            // UPDATE_JOINT_STATE
            SDO_UPDATE_CASE(PositionActualInternalValue);
            SDO_UPDATE_CASE(VelocityActualValue);
            SDO_UPDATE_CASE(CurrentActualValue);
        }
    }

    if (update & UPDATE_FACTORS) {
        try {
            mFactors = computeFactors();
        }
        catch(canopen_master::ObjectNotRead) {}
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

std::vector<canbus::Message> Controller::queryJointState() const
{
    // NOTE: we don't need to query TorqueActualValue. Given how bot this and
    // CurrentActualValue are encoded, they contain the same value
    return vector<canbus::Message> {
        queryObject<PositionActualInternalValue>(),
        queryObject<VelocityActualValue>(),
        queryObject<CurrentActualValue>()
    };
}

base::JointState Controller::getJointState() const
{
    auto position = getRaw<PositionActualInternalValue>();
    auto velocity = getRaw<VelocityActualValue>();
    // See comment in queryJointState
    auto current_and_torque = getRaw<CurrentActualValue>();

    base::JointState state;
    state.position = mFactors.positionToUser(position);
    state.speed    = mFactors.velocityToUser(velocity);
    state.raw      = mFactors.currentToUser(current_and_torque);
    state.effort   = mFactors.torqueToUser(current_and_torque);
    return state;
}