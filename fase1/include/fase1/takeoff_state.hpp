#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;

        drone_->log("STATE: TAKEOFF");

        std::vector<Base> bases_obj = *blackboard.get<std::vector<Base>>("bases");

        finished_bases_ = *blackboard.get<bool>("finished_bases");

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");
        
        
        pos_ = drone_->getLocalPosition();
        goal_ = Eigen::Vector3d({pos_[0], pos_[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos_  = drone_->getLocalPosition();

        if ((pos_-goal_).norm() < 0.10){
            if (finished_bases_)
                return "FINISHED BASES";
            else
                return "NEXT BASE";
        }

        goal_diff = goal_ - pos_;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }

        little_goal = goal_diff + pos_;

        drone_->setLocalPosition(goal_[0], goal_[1], little_goal[2], initial_yaw);
        
        return "";
    }

private:
    bool finished_bases_;
    Drone* drone_;
    Eigen::Vector3d pos_, goal_, goal_diff, little_goal;
    float max_velocity = 1.0;
    float initial_yaw;
};