#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <chrono>

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: LANDING");

        pos = drone->getLocalPosition();

        start_time_ = std::chrono::steady_clock::now();

        drone->log("Descending for 8s.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        airspeed = drone->getAirSpeed();

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();

        if (elapsed_time > 8) {
            return "LANDED";
        }

        drone->setLocalVelocity(0.0, 0.0, 0.5, 0.0);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        //Descend for 1s at 0.4m/s to make sure it is landed
        for (int i = 0 ;i < 10; i++){
            drone->setLocalVelocity(0.0, 0.0, 0.4, 0.0);
            usleep(1e5);
        }

        drone->log("Offboard and arming.");
        drone->toOffboardSync();
        drone->armSync();    
    }

private:
    float airspeed;
    Drone* drone;
    Eigen::Vector3d pos;
    std::chrono::steady_clock::time_point start_time_;
};