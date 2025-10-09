#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/aux/Base.hpp"


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

        this->at_home = *blackboard.get<bool>("at_home");

        this->drone->log("Descending for " + std::to_string(this->landing_timeout) + " s.");
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();

        if (elapsed_time > this->landing_timeout) {
            if(this->at_home)
                return "FINISHED";
            return "LANDED";
        }

        this->drone->setLocalVelocity(0.0, 0.0, this->landing_velocity, 0.0);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = this->drone->getLocalPosition();
        this->vision->publishBaseDetection("confirmed_base", pos);

        auto bases_pousadas_ptr = blackboard.get<std::shared_ptr<std::vector<Base>>>("bases_pousadas");
        
        if(bases_pousadas_ptr == nullptr || *bases_pousadas_ptr == nullptr){
            this->drone->log("ERROR: bases_pousadas is null in LandingState on_exit");
            return;
        }

        auto& bases_pousadas = **bases_pousadas_ptr;
        bases_pousadas.push_back({pos, true});

        this->drone->log("New base {" + std::to_string(bases_pousadas.size()) + "}: " +
                    std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ", " + std::to_string(pos.z()));

        this->drone->log("DEBUG: About to check if bases_pousadas.size() == 6");
        
        if (bases_pousadas.size() == 6){
            this->drone->log("DEBUG: Setting finished_bases to true");
            blackboard.set<bool>("finished_bases", true);
            this->drone->log("Visited all 6 bases");
        }
        
        this->drone->log("DEBUG: Exiting LandingState::on_exit successfully");
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    float known_base_radius;
    float landing_velocity;
    float landing_timeout;
    std::chrono::steady_clock::time_point start_time_;

    bool at_home;
};