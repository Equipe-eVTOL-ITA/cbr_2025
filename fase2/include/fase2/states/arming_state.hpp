#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class ArmingState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

public:
    ArmingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");

        if(this->drone == nullptr) return;

        this->drone->log("STATE: ARMING");

        float home_x = *bb.get<float>("fictual_home_x");
        float home_y = *bb.get<float>("fictual_home_y");
        float home_z = *bb.get<float>("fictual_home_z");
        const Eigen::Vector3d fictual_home = Eigen::Vector3d({home_x, home_y, home_z});
        bb.set<Eigen::Vector3d>("home_position", fictual_home);

        // Attempt to arm the drone
        this->drone->toOffboardSync();
        this->drone->armSync();
        this->drone->setHomePosition(fictual_home);
    }
    
    std::string act(fsm::Blackboard &bb) override {
        (void)bb;
        
        // Check if the drone is armed
        this->drone->log("Checking everything before flight...");
        if (this->drone->getArmingState() == DronePX4::ARMING_STATE::ARMED) {
            this->drone->log("Drone armed successfully.");
            return "ARMED";
        }

        this->drone->log("Failed to arm the drone.");
        return "NOT ARMED";
    }
};