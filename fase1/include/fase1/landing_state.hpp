#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase1.hpp"
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

        this->v_max = *blackboard.get<float>("landing_velocity_max");
        this->v_min = *blackboard.get<float>("landing_velocity_min");
        float align_height = - *blackboard.get<float>("align_height"); // Negative
        float max_base_height = - *blackboard.get<float>("max_base_height"); // Negative

        this->time_constant = (this->v_max - this->v_min) / (align_height - max_base_height);

        double TempoBase = (1/this->time_constant) * std::log(this->v_max/this->v_min);
        double TempoTotal = TempoBase + max_base_height / this->v_min;
        this->timeout_ = TempoTotal + 5.0;
        this->start_time_ = std::chrono::steady_clock::now();


        this->drone->log("Tempo até a Base: " + std::to_string(TempoBase) + " s");
        this->drone->log("Tempo total de pouso: " + std::to_string(TempoTotal) + " s");
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();

        float velocity = this->v_max * std::exp(-this->time_constant * elapsed_time);

        velocity = std::clamp(velocity, this->v_min, this->v_max);

        if (elapsed_time > this->timeout_){
            return "LANDED";
        }

        this->drone->setLocalVelocity(0.0, 0.0, velocity, 0.0);

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Eigen::Vector3d pos = this->drone->getLocalPosition();
        this->vision->publishBaseDetection("confirmed_base", pos);

        auto bases = blackboard.get<std::vector<Base>>("bases");
        bases->push_back({pos, true});

        drone->log("New base {" + std::to_string(bases->size()) + "}: " +
                    std::to_string(pos.x()) + ", " + std::to_string(pos.y()) + ", " + std::to_string(pos.z()));

        auto num_bases = *blackboard.get<float>("num_bases");
        size_t total_bases = static_cast<size_t>(num_bases);

        if (bases->size() == total_bases + 1){  // +1 because home is also in the vector
            blackboard.set<bool>("finished_bases", true);
            drone->log("Visited all " + std::to_string(total_bases) + " bases");
        }
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    float v_max, v_min;
    float time_constant;
    float timeout_;
    std::chrono::steady_clock::time_point start_time_;
};