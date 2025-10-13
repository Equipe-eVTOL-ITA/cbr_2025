#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/aux/vision_fase4.hpp"
#include "fase4/aux/Base.hpp"


class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: LANDING");

        this->landing_velocity = *blackboard.get<float>("landing_velocity");
        this->landing_timeout = *blackboard.get<float>("landing_timeout");
        this->start_time_ = std::chrono::steady_clock::now();

        this->drone->log("Descending for " + std::to_string(this->landing_timeout) + " s.");
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();

        if (elapsed_time > this->landing_timeout) {
            return "LANDED";
        }

        this->drone->setLocalVelocity(0.0, 0.0, this->landing_velocity, 0.0);

        return "";
    }

private:
    std::shared_ptr<Drone> drone;

    float landing_velocity;
    float landing_timeout;
    std::chrono::steady_clock::time_point start_time_;
};