#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <chrono>

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("STATE: RETURN HOME");

        home_pos_ = *blackboard.get<Eigen::Vector3d>("home_position");
        pos_ = drone_->getLocalPosition();
        initial_yaw = *blackboard.get<float>("initial_yaw");

        goal_ = Eigen::Vector3d({home_pos_.x(), home_pos_.y(), pos_.z()});
        
        drone_->log("Going to home at: " + std::to_string(goal_[0]) + " " + std::to_string(goal_[1]));

        start_time_ = std::chrono::steady_clock::now();
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        drone_->log("At home, now entered Land Mode for precaution.");
        drone_->land();
        rclcpp::sleep_for(std::chrono::seconds(5));
        drone_->disarmSync();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        pos_ = drone_->getLocalPosition();

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();

        if (elapsed_time > 15) {
            return "AT HOME";
        }

        if (!over_base_){
            if ((pos_-goal_).norm() < 0.10){
                over_base_ = true;
            }

            goal_diff = goal_ - pos_;
            if (goal_diff.norm() > max_velocity){
                goal_diff = goal_diff.normalized() * max_velocity;
            }
            little_goal_ = goal_diff + pos_;
            
            drone_->setLocalPosition(little_goal_[0], little_goal_[1], pos_[2], initial_yaw);
        }
        else
        {
            if ((pos_ - home_pos_).norm() < 0.10){
                return "AT HOME";
            }

            goal_diff = home_pos_ - pos_;
            if (goal_diff.norm() > max_velocity){
                goal_diff = goal_diff.normalized() * max_velocity;
            }
            little_goal_ = goal_diff + pos_;
            drone_->setLocalPosition(little_goal_[0], little_goal_[1], little_goal_[2] - 0.05, initial_yaw);
        }

        return "";
    }

private:
    bool over_base_ = false;
    Eigen::Vector3d home_pos_, pos_, home_horizon_, goal_, goal_diff, little_goal_;
    Drone* drone_;
    const float max_velocity = 0.7;
    std::chrono::steady_clock::time_point start_time_; 
    float initial_yaw;
};