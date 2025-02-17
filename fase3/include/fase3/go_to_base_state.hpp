#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"

class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: GO TO BASE");

        bases = blackboard.get<std::vector<Base>>("bases");

        orientation = drone->getOrientation();

        for (auto& base : *bases) {
            if (!base.is_visited) {
                base_to_visit = &base;
                drone->log("Going to base " + base.name + " at {" 
                    + std::to_string(base.coordinates[0]) + ", " 
                    + std::to_string(base.coordinates[1]) + ", " 
                    + std::to_string(base.coordinates[2]) + "}");
                finished_bases = false;
                break;
            }
            finished_bases = true;
        }
        goal = base_to_visit->coordinates;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        if (finished_bases) return "FINISHED BASES";
        
        pos = drone->getLocalPosition();

        if ((pos - goal).norm() < 0.10){
            base_to_visit->is_visited = true;
            drone->log("Base approximately at: {" + std::to_string(pos[0]) + ", " 
                + std::to_string(pos[1]) + ", " + std::to_string(pos[2]) + "}");
            return "AT APPROXIMATE BASE";
        }

        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        little_goal = goal_diff + pos;

        drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], orientation[2]);

        return ""; 
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        blackboard.set<Base>("current_base", *base_to_visit);
    }
    
private:
    Drone* drone;
    std::vector<Base>* bases;
    Base* base_to_visit;
    Eigen::Vector3d pos, orientation, goal, goal_diff, little_goal;
    const float max_velocity = 0.4;
    bool finished_bases = false;
};