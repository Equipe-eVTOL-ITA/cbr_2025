#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class EnterIntoHouseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    bool enter_house_aligned;
    bool yaw_aligned_with_entrance;

public:
    EnterIntoHouseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &&bb){
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->enter_house_aligned = *bb.get<bool>("enter_house_aligned");
        this->yaw_aligned_with_entrance = false;
    }

    std::string act(fsm::Blackboard &bb){
        (void) bb;

        if(this->yaw_aligned_with_entrance == false){
            // implementar código para alinhar o yaw
            return "";
        }

        if(this->enter_house_aligned == false)
            return "ALIGN WITH WINDOW";


        return "";
    }

};