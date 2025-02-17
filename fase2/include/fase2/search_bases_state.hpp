#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class SearchBasesState : public fsm::State {
public:
    SearchBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GO TO SHELL");
        orientation = drone->getOrientation();
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        initial_goal = Eigen::Vector3d({7.2, -3.0, takeoff_height});

        //Eigen::Vector3d* last_search_ptr = blackboard.get<Eigen::Vector3d>("last search position");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        
        pos = drone->getLocalPosition();

        if ((pos - initial_goal).norm() < 0.10)
            return "SEARCH ENDED";

        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        goal = goal_diff + pos;
        drone->setLocalPosition(goal[0], goal[1], goal[2], orientation[2]);

        return ""; 
    }

private:
    Drone* drone;
    Eigen::Vector3d pos, goal, goal_diff, orientation, initial_goal;
    const float max_velocity = 0.4;
    
};