#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
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
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();

        if (elapsed_time > 8) {
            return "LANDED";
        }

        drone->setLocalVelocity(0.0, 0.0, 0.5, 0.0);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        //Publish base coordinates
        pos = drone->getLocalPosition();
        std::vector<Base> bases = *blackboard.get<std::vector<Base>>("bases");
        bases.push_back({pos, true});
        blackboard.set<std::vector<Base>>("bases", bases);

        drone->log("New base {" + std::to_string(bases.size()) + "}: " +
                    std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ", " + std::to_string(pos.z()));

        if (bases.size() > 5){
            blackboard.set<bool>("finished_bases", true);
            drone->log("Visited all 6 bases");
        }

        drone->log("Offboard and arming.");
        drone->toOffboardSync();
        drone->armSync();    
    }

private:
    Drone* drone;
    Eigen::Vector3d pos;
    std::chrono::steady_clock::time_point start_time_;
};