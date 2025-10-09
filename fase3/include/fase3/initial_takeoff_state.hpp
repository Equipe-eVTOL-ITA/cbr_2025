#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) {
            return;
        }

        this->drone->log("");
        this->drone->log("STATE: INITIAL TAKEOFF");

        const float home_x = *blackboard.get<float>("fictual_home_x");
        const float home_y = *blackboard.get<float>("fictual_home_y");
        const float home_z = *blackboard.get<float>("fictual_home_z");
        const Eigen::Vector3d home_position({home_x, home_y, home_z});
        blackboard.set<Eigen::Vector3d>("home_position", home_position);

        const bool already_armed = this->drone->getArmingState() == DronePX4::ARMING_STATE::ARMED;
        if (!already_armed) {
            this->drone->toOffboardSync();
            this->drone->armSync();
        }

        this->drone->setHomePosition(home_position);

        this->max_velocity = *blackboard.get<float>("max_vertical_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        const float takeoff_height = *blackboard.get<float>("takeoff_height");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d(this->pos.x(), this->pos.y(), takeoff_height);

        this->print_counter = 0;
        this->drone->log("Home position set to: " + std::to_string(home_position.x()) + ", " +
                         std::to_string(home_position.y()) + ", " + std::to_string(home_position.z()));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (this->drone == nullptr) {
            return "SEG FAULT";
        }

        if (this->print_counter % 10 == 0) {
            this->drone->log("Takeoff target: {" + std::to_string(this->goal.x()) + ", " +
                              std::to_string(this->goal.y()) + ", " + std::to_string(this->goal.z()) + "}");
        }
        ++this->print_counter;

        this->pos = this->drone->getLocalPosition();
        const Eigen::Vector3d diff = this->goal - this->pos;

        if (diff.norm() < this->position_tolerance) {
            this->drone->log("Reached takeoff height");
            blackboard.set<float>("hover_height", static_cast<float>(this->goal.z()));
            return "INITIAL TAKEOFF COMPLETED";
        }

        const Eigen::Vector3d step = diff.norm() > this->max_velocity ? diff.normalized() * this->max_velocity : diff;
        const Eigen::Vector3d little_goal = this->pos + step;

        this->drone->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), this->initial_yaw);
        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    float max_velocity{};
    float position_tolerance{};
    float initial_yaw{};
    int print_counter{0};
};