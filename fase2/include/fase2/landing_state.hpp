#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"
#include "Base.hpp"


class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if (this->vision == nullptr || this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: LANDING");

        this->known_base_radius = *blackboard.get<float>("known_base_radius");
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

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = this->drone->getLocalPosition();
        this->vision->publishBaseDetection("confirmed_base", pos.head<2>(), pos.z());

        auto bases = blackboard.get<std::vector<Base>>("bases");
        bases->push_back({pos, true});

        drone->log("New base {" + std::to_string(bases->size()) + "}: " +
                    std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ", " + std::to_string(pos.z()));

        if (bases->size() == 7){
            blackboard.set<bool>("finished_bases", true);
            drone->log("Visited all 6 bases");
        }
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    float known_base_radius;
    float landing_velocity;
    float landing_timeout;
    std::chrono::steady_clock::time_point start_time_;
};