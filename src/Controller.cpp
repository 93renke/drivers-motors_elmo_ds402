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

canbus::Message Controller::querySync() const
{
    return mCanOpen.sync();
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
        queryObject<AccelerationFactorNum>(),
        queryObject<AccelerationFactorDen>(),
        queryObject<GearRatioNum>(),
        queryObject<GearRatioDen>(),
        queryObject<FeedConstantNum>(),
        queryObject<FeedConstantDen>(),
        queryObject<VelocityFactorNum>(),
        queryObject<VelocityFactorDen>(),
        queryObject<MotorRatedCurrent>(),
        queryObject<MotorRatedTorque>()
    };
}

Factors Controller::getFactors() const
{
    return mFactors;
}

Factors Controller::computeFactors() const
{
    Factors factors;
    factors.encoderTicks = getRaw<PositionEncoderResolutionNum>();
    factors.encoderRevolutions = getRaw<PositionEncoderResolutionDen>();
    factors.gearMotorShaftRevolutions = getRaw<GearRatioNum>();
    factors.gearDrivingShaftRevolutions = getRaw<GearRatioDen>();
    factors.feedLength = getRaw<FeedConstantNum>();
    factors.feedDrivingShaftRevolutions = getRaw<FeedConstantDen>();
    factors.ratedTorque  = static_cast<double>(getRaw<MotorRatedTorque>()) / 1000;
    factors.ratedCurrent = static_cast<double>(getRaw<MotorRatedCurrent>()) / 1000;
    return factors;
}

#define MODE_UPDATE_CASE(mode, object) \
    case canopen_master::StateMachine::mode: \
        update |= object::UPDATE_ID; \
        break;

#define SDO_UPDATE_CASE(object) \
    case (static_cast<uint32_t>(object::OBJECT_ID) << 8 | object::OBJECT_SUB_ID): \
        update |= object::UPDATE_ID; \
        break;

Update Controller::process(canbus::Message const& msg)
{
    uint64_t update = 0;
    auto canUpdate = mCanOpen.process(msg);
    switch(canUpdate.mode)
    {
        MODE_UPDATE_CASE(PROCESSED_HEARTBEAT, Heartbeat);
        case canopen_master::StateMachine::PROCESSED_SDO_INITIATE_DOWNLOAD:
        {
            // Ack of a upload request
            auto object = canUpdate.updated[0];
            return Update::Ack(object.first, object.second);
        }

        default: ; // we just ignore the rest, we really don't care
    };

    for (auto it = canUpdate.begin(); it != canUpdate.end(); ++it)
    {
        uint32_t fullId = static_cast<uint32_t>(it->first) << 8 | it->second;

        switch(fullId)
        {
            SDO_UPDATE_CASE(StatusWord);

            // UPDATE_FACTORS
            SDO_UPDATE_CASE(PositionEncoderResolutionNum);
            SDO_UPDATE_CASE(PositionEncoderResolutionDen);
            SDO_UPDATE_CASE(VelocityEncoderResolutionNum);
            SDO_UPDATE_CASE(VelocityEncoderResolutionDen);
            SDO_UPDATE_CASE(AccelerationFactorNum);
            SDO_UPDATE_CASE(AccelerationFactorDen);
            SDO_UPDATE_CASE(GearRatioNum);
            SDO_UPDATE_CASE(GearRatioDen);
            SDO_UPDATE_CASE(FeedConstantNum);
            SDO_UPDATE_CASE(FeedConstantDen);
            SDO_UPDATE_CASE(VelocityFactorNum);
            SDO_UPDATE_CASE(VelocityFactorDen);
            SDO_UPDATE_CASE(MotorRatedCurrent);
            SDO_UPDATE_CASE(MotorRatedTorque);

            // UPDATE_JOINT_STATE
            SDO_UPDATE_CASE(PositionActualInternalValue);
            SDO_UPDATE_CASE(VelocityActualValue);
            SDO_UPDATE_CASE(CurrentActualValue);

            // UPDATE_JOINT_LIMITS
            SDO_UPDATE_CASE(SoftwarePositionLimitMin);
            SDO_UPDATE_CASE(SoftwarePositionLimitMax);
            SDO_UPDATE_CASE(MaxMotorSpeed);
            SDO_UPDATE_CASE(MaxAcceleration);
            SDO_UPDATE_CASE(MaxDeceleration);
            SDO_UPDATE_CASE(MaxCurrent);
        }
    }

    if (update & UPDATE_FACTORS) {
        try {
            mFactors = computeFactors();
        }
        catch(canopen_master::ObjectNotRead) {}
    }

    return Update::UpdatedObjects(update);
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
    state.position = mFactors.scaleEncoderValue(position);
    state.speed    = mFactors.scaleEncoderValue(velocity);
    state.raw      = mFactors.currentToUser(current_and_torque);
    state.effort   = mFactors.currentToUserTorque(current_and_torque);
    return state;
}

vector<canbus::Message> Controller::queryJointLimits() const
{
    return vector<canbus::Message> {
        queryObject<SoftwarePositionLimitMin>(),
        queryObject<SoftwarePositionLimitMax>(),
        queryObject<MaxMotorSpeed>(),
        queryObject<MaxAcceleration>(),
        queryObject<MaxDeceleration>(),
        queryObject<MaxCurrent>()
    };
}

base::JointLimitRange Controller::getJointLimits() const
{
    base::JointState min;
    base::JointState max;

    int32_t rawPositionMin = getRaw<SoftwarePositionLimitMin>();
    int32_t rawPositionMax = getRaw<SoftwarePositionLimitMax>();
    if (rawPositionMin == rawPositionMax && rawPositionMin == 0)
    {
        min.position = -base::infinity<double>();
        max.position = base::infinity<double>();
    }
    else
    {
        min.position = mFactors.scaleEncoderValue(rawPositionMin);
        max.position = mFactors.scaleEncoderValue(rawPositionMax);
    }

    int32_t rawMaxSpeed = getRaw<MaxMotorSpeed>();
    if (rawMaxSpeed < 0)
    {
        min.speed = -base::infinity<double>();
        max.speed = base::infinity<double>();
    }
    else
    {
        double speedLimit = mFactors.scaleEncoderValue(rawMaxSpeed);
        min.speed = -speedLimit;
        max.speed = speedLimit;
    }

    min.acceleration = -base::infinity<double>();
    max.acceleration = base::infinity<double>();

    auto torqueAndCurrentLimit = getRaw<MaxCurrent>();
    double torqueLimit = mFactors.currentToUserTorque(torqueAndCurrentLimit);
    min.effort = -torqueLimit;
    max.effort = torqueLimit;

    double currentLimit = mFactors.currentToUser(torqueAndCurrentLimit);
    min.raw = -currentLimit;
    max.raw = currentLimit;

    base::JointLimitRange range;
    range.min = min;
    range.max = max;
    return range;
}

struct PDOMapping : canopen_master::PDOMapping
{
    template<typename Object>
    void add()
    {
        canopen_master::PDOMapping::add(
            Object::OBJECT_ID, Object::OBJECT_SUB_ID, sizeof(typename Object::OBJECT_TYPE));
    }
};

vector<canbus::Message> Controller::queryPeriodicJointStateUpdate(
    int pdoIndex, base::Time const& period)
{
    canopen_master::PDOCommunicationParameters parameters;
    parameters.transmission_mode = canopen_master::PDO_SYNCHRONOUS;
    parameters.timer_period = period;

    PDOMapping mapping0;
    mapping0.add<PositionActualInternalValue>();
    mapping0.add<VelocityActualValue>();
    auto pdo0 = mCanOpen.configurePDO(true, pdoIndex, parameters, mapping0);

    PDOMapping mapping1;
    mapping1.add<CurrentActualValue>();
    auto pdo1 = mCanOpen.configurePDO(true, pdoIndex + 1, parameters, mapping1);

    mCanOpen.declarePDOMapping(pdoIndex, mapping0);
    mCanOpen.declarePDOMapping(pdoIndex + 1, mapping1);

    vector<canbus::Message> messages;
    messages.insert(messages.end(), pdo0.begin(), pdo0.end());
    messages.insert(messages.end(), pdo1.begin(), pdo1.end());
    return messages;
}

canbus::Message Controller::querySave()
{
    uint8_t buffer[4] = { 's', 'a', 'v', 'e' };
    return mCanOpen.download(0x1010, 1, buffer, 4);
}

canbus::Message Controller::queryLoad()
{
    uint8_t buffer[4] = { 'l', 'o', 'a', 'd' };
    return mCanOpen.download(0x1011, 1, buffer, 4);
}