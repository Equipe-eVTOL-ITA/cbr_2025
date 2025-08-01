#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase1.hpp"


class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: RETURN HOME");

        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->takeoff_height = *blackboard.get<float>("takeoff_height");

        auto home_pos = *blackboard.get<Eigen::Vector3d>("home_position");
        this->initial_yaw = this->drone->getOrientation()[2];

        this->goal = Eigen::Vector3d(
            home_pos.x(),
            home_pos.y(),
            this->takeoff_height
        );
        
        this->drone->log("Going to home at: " + std::to_string(this->goal.x()) + " " + std::to_string(this->goal.y()));

        this->start_time = std::chrono::steady_clock::now();
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        this->drone->log("At home, now entered Land Mode for precaution.");
        this->drone->land();
        rclcpp::sleep_for(std::chrono::seconds(5));
        this->drone->disarmSync();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        Eigen::Vector3d pos = this->drone->getLocalPosition();

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - this->start_time).count();

        if (elapsed_time > 15) {
            return "AT HOME";
        }

        // First phase: Move to home position horizontally at current altitude
        if (!this->over_base) {
            Eigen::Vector3d horizontal_goal = Eigen::Vector3d(this->goal.x(), this->goal.y(), pos.z());
            Eigen::Vector3d diff = horizontal_goal - pos;
            
            if (diff.norm() < this->position_tolerance) {
                this->over_base = true;
                return "";
            }

            Eigen::Vector3d little_goal = pos + (diff.norm() > this->max_velocity ?
                                                diff.normalized() * this->max_velocity : diff);

            this->drone->setLocalPosition(
                little_goal.x(),
                little_goal.y(),
                little_goal.z(),
                this->initial_yaw
            );
        }
        // Second phase: Descend to home position
        else {
            Eigen::Vector3d diff = this->goal - pos;
            
            if (diff.norm() < this->position_tolerance) {
                return "AT HOME";
            }

            Eigen::Vector3d little_goal = pos + (diff.norm() > this->max_velocity ?
                                                diff.normalized() * this->max_velocity : diff);

            this->drone->setLocalPosition(
                little_goal.x(),
                little_goal.y(),
                little_goal.z(),
                this->initial_yaw
            );
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    float max_velocity;
    float position_tolerance;
    float takeoff_height;
    float initial_yaw;
    Eigen::Vector3d goal;
    bool over_base = false;
    std::chrono::steady_clock::time_point start_time;
};