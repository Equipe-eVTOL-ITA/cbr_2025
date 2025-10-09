#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class PackageState : public fsm::State {
public:
    PackageState() : fsm::State() {}

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        return "DONE";
    }
};