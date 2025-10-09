#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/aux/movement.hpp"

class ApproachState : public fsm::State {
protected:
    std::shared_ptr<Drone> drone;

    float velocity;
    float approach_height;
    float tolerance;

    bool reached;

    virtual void setApproachParams(fsm::Blackboard &bb) = 0;

public:
    ApproachState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override final {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->setApproachParams(bb);

        reached = false;

        this->drone->log("");
        this->drone->log("STATE: APPROACH TO DELIVER");
    }


    std::string act(fsm::Blackboard &bb) override {
        (void) bb;
        
        Eigen::Vector3d pos = this->drone->getLocalPosition();
        Eigen::Vector3d goal = Eigen::Vector3d({pos[0], pos[1], this->approach_height}); // Altura de entrega fixa

        reached = move_local_by_waypoint(this->drone, goal, this->velocity, this->tolerance);

        Eigen::Vector3d error = goal - pos;

        this->drone->log("Goal: " + std::to_string(goal.z())
                        + " Pos: " + std::to_string(pos.z())
                        + " Error: " + std::to_string(error.z()));

        if(reached) {
            this->drone->log("Reached delivery approach point.");
            return "REACHED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &bb) override {
        (void) bb;
        this->drone->log("Exiting approach state");
    }
};