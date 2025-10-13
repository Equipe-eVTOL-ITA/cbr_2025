#include "fsm/fsm.hpp"
#include <Eigen/Eigen>
#include "drone/Drone.hpp"
#include "fase4/aux/movement.hpp"

class LookToWindowState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    Eigen::Vector3d window_position;
    bool looking_completed = false;

public:
    LookToWindowState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->looking_completed = false;

        this->window_position = *bb.get<Eigen::Vector3d>("window_position");

        this->drone->log("STATE: LOOK TO WINDOW STATE");

    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        looking_completed = lookToPoint(this->drone, this->window_position, 0.3f, 0.1f);

        if(looking_completed)
            return "LOOKING";

        return "";
    }

};