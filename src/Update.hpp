#ifndef MOTORS_ELMO_DS402_UPDATE_HPP
#define MOTORS_ELMO_DS402_UPDATE_HPP

#include <cstdint>

namespace motors_elmo_ds402
{
    class Update
    {
        uint32_t mAckedObjectID;
        uint32_t mAckedObjectSubID;
        uint64_t mUpdatedObjects;

    public:
        static Update Ack(int objectId, int objectSubId)
        {
            Update update;
            update.mAckedObjectID = objectId;
            update.mAckedObjectSubID = objectSubId;
            return update;
        }

        static Update UpdatedObjects(uint64_t updates)
        {
            Update update;
            update.mUpdatedObjects = updates;
            return update;

        }
        Update()
            : mAckedObjectID(0)
            , mAckedObjectSubID(0)
            , mUpdatedObjects(0) {}

        bool isAck() const
        {
            return mAckedObjectID != 0;
        }

        bool isAcked(uint16_t objectId, uint8_t objectSubID)
        {
            return (mAckedObjectID == objectId) &&
                (mAckedObjectSubID == objectSubID);
        }

        template<typename T>
        bool isUpdated() const
        {
            return isUpdated(T::UPDATE_ID);
        }

        bool isUpdated(uint64_t updateId)
        {
            return (mUpdatedObjects & updateId);
        }
    };
}

#endif

