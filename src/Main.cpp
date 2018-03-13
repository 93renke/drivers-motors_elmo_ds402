#include <iostream>
#include <canbus.hh>
#include <memory>
#include <motors_elmo_ds402/Controller.hpp>
#include <string>

using namespace std;
using namespace motors_elmo_ds402;

int usage()
{
    cout << "motors_elmo_ds402_ctl CAN_DEVICE CAN_DEVICE_TYPE CAN_ID COMMAND\n";
    cout << "  get-state # displays the drive's internal state\n";
    cout << endl;
    return 1;
}

const char* stateToString(StatusWord::State state)
{
    switch(state)
    {
        case StatusWord::NOT_READY_TO_SWITCH_ON: return "NOT_READY_TO_SWITCH_ON";
        case StatusWord::SWITCH_ON_DISABLED: return "SWITCH_ON_DISABLED";
        case StatusWord::READY_TO_SWITCH_ON: return "READY_TO_SWITCH_ON";
        case StatusWord::SWITCH_ON: return "SWITCH_ON";
        case StatusWord::OPERATION_ENABLED: return "OPERATION_ENABLED";
        case StatusWord::QUICK_STOP_ACTIVE: return "QUICK_STOP_ACTIVE";
        case StatusWord::FAULT_REACTION_ACTIVE: return "FAULT_REACTION_ACTIVE";
        case StatusWord::FAULT: return "FAULT";
        default:
            throw std::invalid_argument("unknown state");
    }
}

template<typename ExpectedUpdate>
static void sendAndWait(canbus::Driver& device, canbus::Message const& query,
    motors_elmo_ds402::Controller& controller,
    base::Time timeout = base::Time::fromMilliseconds(100))
{
    device.write(query);
    device.setReadTimeout(timeout.toMilliseconds());
    while(true)
    {
        canbus::Message msg = device.read();
        if (controller.process(msg).isUpdated<ExpectedUpdate>()) {
            return;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 5) {
        return usage();
    }

    std::string can_device(argv[1]);
    std::string can_device_type(argv[2]);
    int8_t node_id(stoi(argv[3]));
    std::string cmd(argv[4]);

    unique_ptr<canbus::Driver> device(canbus::openCanDevice(can_device, can_device_type));
    Controller controller(node_id);

    if (cmd == "reset")
    {
        sendAndWait<Heartbeat>(*device,
            controller.queryNodeStateTransition(canopen_master::NODE_RESET),
            controller, base::Time::fromMilliseconds(5000));
        controller.getNodeState();
    }
    else if (cmd == "get-state")
    {
        sendAndWait<Heartbeat>(*device, controller.queryNodeState(), controller);
        auto nodeState = controller.getNodeState();
        std::cout << "Node state: " << nodeState << std::endl;

        sendAndWait<StatusWord>(*device, controller.queryStatusWord(), controller);
        StatusWord status = controller.getStatusWord();
        std::cout << stateToString(status.state) << std::endl;
    }
    return 0;
}
