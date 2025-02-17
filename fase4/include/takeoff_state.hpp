#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: TAKEOFF");
        
        pos = drone->getLocalPosition();
        orientation = drone->getOrientation();
        
        float takeoff_height = *blackboard.get<float>("takeoff height");

        goal = Eigen::Vector3d({pos[0], pos[1], takeoff_height});
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        pos  = drone->getLocalPosition();

        if ((pos-goal).norm() < 0.15)
            return "TAKEOFF COMPLETED";

        goal_diff = goal - pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }

        little_goal = goal_diff + pos;

        drone->setLocalPosition(goal[0], goal[1], little_goal[2], orientation[2]);
        
        return "";
    }

private:
    float max_velocity = 1.5;
    Eigen::Vector3d pos, orientation, goal, goal_diff, little_goal;
    Drone* drone;
};