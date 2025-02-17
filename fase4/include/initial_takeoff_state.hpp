#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

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
        orientation = drone->getOrientation();
        
        float* takeoff_height = blackboard.get<float>("takeoff height");

        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

        goal = Eigen::Vector3d({pos[0], pos[1], *takeoff_height});
        drone->log("Goal is height: " + std::to_string(goal[2]));
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
        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], orientation[2]);
        
        return "";
    }

private:
    const float max_velocity = 1.0;
    Eigen::Vector3d pos, orientation, goal, goal_diff, little_goal;
    Drone* drone;
    int i_ = 0;
};