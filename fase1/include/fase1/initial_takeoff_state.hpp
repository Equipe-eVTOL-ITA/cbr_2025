#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "ArenaPoint.hpp"

class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        drone->toOffboardSync();
        drone->armSync();
        
        pos = drone->getLocalPosition();
        float target_height = *blackboard.get<float>("takeoff_height");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], target_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if (i_%10==0){
            drone->log("Pos: {" + std::to_string(pos[0]) + ", " 
                        + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "}");
        }
        i_++;

        if ((pos-goal).norm() < 0.15){
            return "INITIAL TAKEOFF COMPLETED";
        }

        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        little_goal = goal_diff + pos;
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);
        
        return "";
    }

private:
    const float max_velocity = 1.0;
    Eigen::Vector3d pos, goal, goal_diff, little_goal;
    Drone* drone;
    int i_ = 0;
    float initial_yaw;
};