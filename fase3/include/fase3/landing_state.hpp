#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: LANDING");

        this->v_max = *blackboard.get<float>("landing_velocity_max");
        this->v_min = *blackboard.get<float>("landing_velocity_min");
        
        // Get current drone height as starting point
        float current_height = this->drone->getLocalPosition().z();
        float align_height = - current_height; // Current height (negative in FRD)
        float max_base_height = - *blackboard.get<float>("max_base_height"); // Negative

        this->drone->log("Altura atual: " + std::to_string(align_height));

        this->time_constant = (this->v_max - this->v_min) / (align_height - max_base_height);

        double TempoBase = (1/this->time_constant) * std::log(this->v_max/this->v_min);
        double TempoTotal = TempoBase + max_base_height / this->v_min;
        
        float landing_timeout_margin = *blackboard.get<float>("landing_timeout");
        this->timeout_ = TempoTotal + landing_timeout_margin;
        this->start_time_ = std::chrono::steady_clock::now();

        this->drone->log("Tempo até a Base: " + std::to_string(TempoBase) + " s");
        this->drone->log("Tempo total de pouso: " + std::to_string(TempoTotal) + " s");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        
        if (this->drone == nullptr) {
            return "SEG FAULT";
        }
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time_).count();

        float velocity = this->v_max * std::exp(-this->time_constant * elapsed_time);

        velocity = std::clamp(velocity, this->v_min, this->v_max);

        if (elapsed_time > this->timeout_){
            this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            this->drone->log("Landing completed");
            return "LANDED";
        }

        this->drone->setLocalVelocity(0.0, 0.0, velocity, 0.0);

        return "";
    }

private:
    std::shared_ptr<Drone> drone;

    float v_max, v_min;
    float time_constant;
    float timeout_;
    std::chrono::steady_clock::time_point start_time_;
};