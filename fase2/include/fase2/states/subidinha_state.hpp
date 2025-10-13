#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class SubidinhaState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

public:
    SubidinhaState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;
        // subindo um pouco para tentar achar a base
        Eigen::Vector3d new_pos = this->drone->getLocalPosition();
        new_pos.z() -= 0.2f; // sobe 20 cm
        move_local_by_waypoint(this->drone, new_pos, 0.1f);

        return "SUBI";
    }
};