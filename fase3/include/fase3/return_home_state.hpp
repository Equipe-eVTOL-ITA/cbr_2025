#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) {
            return;
        }

        this->drone->log("");
        this->drone->log("STATE: RETURN HOME");

        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->max_vertical_velocity = *blackboard.get<float>("max_vertical_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->timeout = *blackboard.get<float>("return_home_timeout");

        const Eigen::Vector3d home_position = *blackboard.get<Eigen::Vector3d>("home_position");
        this->takeoff_height = *blackboard.get<float>("takeoff_height");

        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d(home_position.x(), home_position.y(), this->takeoff_height);
        this->over_base = false;
        this->start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (this->drone == nullptr) {
            return "SEG FAULT";
        }

        const Eigen::Vector3d pos = this->drone->getLocalPosition();

        const auto now = std::chrono::steady_clock::now();
        const float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - this->start_time).count();
        if (elapsed >= this->timeout) {
            this->drone->log("Return home timeout reached, assuming success");
            return "AT HOME";
        }

        if (!this->over_base) {
            const Eigen::Vector3d horizontal_goal(this->goal.x(), this->goal.y(), pos.z());
            const Eigen::Vector3d diff = horizontal_goal - pos;

            if (diff.norm() < this->position_tolerance) {
                this->over_base = true;
            } else {
                const Eigen::Vector3d step = diff.norm() > this->max_velocity ?
                                             diff.normalized() * this->max_velocity : diff;
                const Eigen::Vector3d little_goal = pos + step;
                this->drone->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), this->initial_yaw);
                return "";
            }
        }

        const Eigen::Vector3d diff = this->goal - pos;

        if (diff.norm() < this->position_tolerance) {
            this->drone->log("Reached home position");
            return "AT HOME";
        }

        const float step_size = std::min(this->max_vertical_velocity, static_cast<float>(diff.norm()));
        const Eigen::Vector3d step = diff.normalized() * step_size;
        const Eigen::Vector3d little_goal = pos + step;
        this->drone->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), this->initial_yaw);
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (this->drone != nullptr) {
            this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            this->drone->land();
            this->drone->disarmSync();
        }
    }

private:
    std::shared_ptr<Drone> drone;
    Eigen::Vector3d goal;
    bool over_base{false};
    float max_velocity{};
    float max_vertical_velocity{};
    float position_tolerance{};
    float timeout{};
    float takeoff_height{};
    float initial_yaw{};
    std::chrono::steady_clock::time_point start_time{};
};