#include "fsm/fsm.hpp"

void setLastState(fsm::Blackboard &bb, std::string last_state){
    bb.set<std::string>("last_state", last_state);
}

class ReturnToLastStateState : public fsm::State {
public:
    ReturnToLastStateState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb){
        this->last_state = *bb.get<std::string>("last_state");
    }

    std::string act(fsm::Blackboard &bb){
        (void) bb;
        return this->last_state;
    }

private:
    std::string last_state;

};