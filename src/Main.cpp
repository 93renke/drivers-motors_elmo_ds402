#include <iostream>
#include <canbus.hh>
#include <memory>
#include <motors_elmo_ds402/Controller.hpp>
#include <string>

using namespace std;
using namespace motors_elmo_ds402;

int usage()
{
    cout << "motors_elmo_ds402_ctl CAN_DEVICE CAN_ID COMMAND" << std::endl;
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

int main(int argc, char** argv)
{
    if (argc <= 4) {
        return usage();
    }

    std::string can_device(argv[1]);
    int8_t node_id(stoi(argv[2]));
    std::string cmd(argv[3]);

    unique_ptr<canbus::Driver> device(canbus::openCanDevice(can_device));
    Controller drive_controller(node_id);

    if (cmd == "get-state")
    {
        canbus::Message queryMsg = drive_controller.queryStatusWord();
        device->write(queryMsg);
        while(true)
        {
            canbus::Message msg = device->read();
            if (drive_controller.process(msg).isUpdated<StatusWord>()) {
                break;
            }
        }

        StatusWord status = drive_controller.getStatusWord();
        std::cout << stateToString(status.state) << std::endl;
    }
    return 0;
}
