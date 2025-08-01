#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"


class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) return;
        this->drone->log("STATE: LANDING");


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

        this->drone->setLocalVelocity(0.0, 0.0, 0.5, 0.0);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = drone->getLocalPosition();

        auto bases = blackboard.get<std::vector<Base>>("bases");
        bases->push_back({pos, true});

        drone->log("New base {" + std::to_string(bases->size()) + "}: " +
                    std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ", " + std::to_string(pos.z()));

        if (bases->size() > 5){
            blackboard.set<bool>("finished_bases", true);
            drone->log("Visited all 6 bases");
        }
    }

private:
    std::shared_ptr<Drone> drone;
    std::chrono::steady_clock::time_point start_time_;
    float landing_timeout;
};