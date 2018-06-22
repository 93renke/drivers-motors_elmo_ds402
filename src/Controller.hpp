#ifndef MOTORS_ELMO_DS402_CONTROLLER_HPP
#define MOTORS_ELMO_DS402_CONTROLLER_HPP

#include <canopen_master/StateMachine.hpp>
#include <motors_elmo_ds402/Objects.hpp>
#include <motors_elmo_ds402/Update.hpp>
#include <motors_elmo_ds402/Factors.hpp>
#include <motors_elmo_ds402/MotorParameters.hpp>
#include <base/JointState.hpp>
#include <base/JointLimitRange.hpp>

namespace motors_elmo_ds402 {
    struct HasPendingQuery : public std::runtime_error {};

    /** Representation of a controller through the CANOpen protocol
     *
     * This is designed to be independent of _how_ the CAN bus
     * itself is being accessed. It represents only the protocol
     */

    class Controller
    {
        typedef canopen_master::StateMachine StateMachine;

    public:
        Controller(uint8_t nodeId);

        /** Give the motor rated torque
         *
         * This is necessary to use torque commands and status
         */
        void setRatedTorque(double torque);

        /** Returns the motor rated torque
         */
        double getRatedTorque() const;

        /** Create a Sync message */
        canbus::Message querySync() const;

        /** Query the canopen node state */
        canbus::Message queryNodeState() const;

        /** Return the last known node state */
        canbus::Message queryNodeStateTransition(
            canopen_master::NODE_STATE_TRANSITION transition) const;

        /** Return the last known node state */
        canopen_master::NODE_STATE getNodeState() const;

        /**
         * Message to query the current status word
         */
        canbus::Message queryStatusWord() const;

        /**
         * Return the last received status word
         */
        StatusWord getStatusWord() const;

        /** Return the set of SDO upload queries that allow
         * to update the factor objects
         */
        std::vector<canbus::Message> queryFactors();

        /**
         * Returns the conversion factor object between Elmo's internal units
         * and physical units
         *
         * This is a cached object that is updated every time the corresponding
         * SDOs are uploaded. All factors can be queried by sending the messages
         * returned by queryFactors.
         */
        Factors getFactors() const;

        /** Explicitely sets motor parameters
         *
         * The CANOpen objects that store the factors are not saved to non-volatile
         * memory. They are therefore basically useless for autoconfiguration
         *
         * The only one that is saved is the rated current. It is actually
         * important to get it from the drive, as the current values are
         * expressed in thousands of this value. Torque has then to be computed
         * from the current value.
         *
         * This allows to set the parameters that can't be extracted from the
         * drive, updating the internal factors in the process. One usually
         * wants to read the factors from the drive beforehand with queryFactors().
         */
        void setMotorParameters(MotorParameters const& parameters);

        /** Return the set of SDO upload queries that allow
         * to update the joint state
         */
        std::vector<canbus::Message> queryJointState() const;

        /**
         * Reads the factor objects from the object dictionary and return them
         */
        base::JointState getJointState(uint64_t fields = UPDATE_JOINT_STATE) const;

        /** Returns the set of SDO upload queries that allow
         * to get the current joint limits
         */
        std::vector<canbus::Message> queryJointLimits() const;

        /**
         * Reads the joint limits from the object dictionary and return them
         */
        base::JointLimitRange getJointLimits() const;

        /**
         * Sets the joint limits and return the set of messages necessary to
         * change them on the drive
         */
        std::vector<canbus::Message> setJointLimits(base::JointLimitRange const& limits);

        /**
         * Configure the controller to periodically send joint state information
         */
        std::vector<canbus::Message> queryPeriodicJointStateUpdate(
            int pdoIndex, canopen_master::PDOCommunicationParameters, uint64_t fields);

        /** @overload */
        std::vector<canbus::Message> queryPeriodicJointStateUpdate(
            int pdoIndex, base::Time const& period, uint64_t fields = UPDATE_JOINT_STATE);

        /** @overload */
        std::vector<canbus::Message> queryPeriodicJointStateUpdate(
            int pdoIndex, int syncPeriod, uint64_t fields = UPDATE_JOINT_STATE);

        template<typename T>
        canbus::Message send(T const& object)
        {
            return mCanOpen.download(T::OBJECT_ID, T::OBJECT_SUB_ID,
                encode<T, typename T::OBJECT_TYPE>(object));
        }

        /** Process a can message and returns what got updated
         */
        Update process(canbus::Message const& msg);

        /** Save configuration to non-volatile memory */
        canbus::Message querySave();

        /** Load configuration from non-volatile memory */
        canbus::Message queryLoad();

    private:
        StateMachine mCanOpen;
        double mRatedTorque;
        Factors mFactors;

        Factors computeFactors() const;

        template<typename T>
        canbus::Message queryObject() const;
        template<typename T> T get() const;
        template<typename T> typename T::OBJECT_TYPE getRaw() const;
        template<typename T> void setRaw(typename T::OBJECT_TYPE value);
    };
}

#endif
