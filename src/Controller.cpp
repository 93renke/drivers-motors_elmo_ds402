#include <motors_elmo_ds402/Controller.hpp>

using namespace motors_elmo_ds402;


Controller::Controller(uint8_t nodeId)
    : mCanOpen(nodeId)
{
}

canbus::Message Controller::queryStatusWord() const
{
    return mCanOpen.upload(StatusWord::OBJECT_ID, 0);
}

Update Controller::process(canbus::Message const& msg)
{
    uint64_t update = 0;
    auto canUpdate = mCanOpen.process(msg);
    for (auto it = canUpdate.begin(); it != canUpdate.end(); ++it)
    {
        switch(it->first)
        {
            case StatusWord::OBJECT_ID:
                update |= StatusWord::UPDATE_ID;
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
