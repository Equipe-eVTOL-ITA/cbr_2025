#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"

class SearchState : public fsm::State {
public:
    SearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) {
            return;
        }

        this->drone->log("");
        this->drone->log("STATE: SEARCH");

        this->yaw_speed_ = *blackboard.get<float>("yaw_speed");
        this->yaw_range_ = *blackboard.get<float>("search_yaw_range");

        const float current_yaw = static_cast<float>(this->drone->getOrientation()[2]);
        this->min_yaw_ = current_yaw - this->yaw_range_;
        this->max_yaw_ = current_yaw + this->yaw_range_;
        this->initial_yaw_ = current_yaw;
        this->clockwise_ = true;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (this->drone == nullptr) {
            return "SEG FAULT";
        }

        const float yaw = static_cast<float>(this->drone->getOrientation()[2]);
        if (yaw >= this->max_yaw_) {
            this->clockwise_ = false;
        } else if (yaw <= this->min_yaw_) {
            this->clockwise_ = true;
        }

        const std::vector<std::string> gestures = this->drone->getHandGestures();
        if (!gestures.empty() && gestures.front() == "Open_Palm") {
            this->drone->log("Hand detected during search");
            return "HAND FOUND";
        }

        const float yaw_rate = this->clockwise_ ? this->yaw_speed_ : -this->yaw_speed_;
        this->drone->setLocalVelocity(0.0, 0.0, 0.0, yaw_rate);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (this->drone != nullptr) {
            this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        }
    }

private:
    std::shared_ptr<Drone> drone;
    float yaw_speed_{};
    float yaw_range_{};
    float min_yaw_{};
    float max_yaw_{};
    float initial_yaw_{};
    bool clockwise_{true};
};
