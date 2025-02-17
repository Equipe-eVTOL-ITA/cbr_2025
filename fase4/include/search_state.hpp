#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <iostream>
#include <vector>
#include <string>

class SearchState : public fsm::State {
public:
    SearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: SEARCH");

        yaw_speed = *blackboard.get<float>("yaw speed");

        initial_yaw = (drone->getOrientation())[2];
        max_yaw = initial_yaw + 3.14/3; // 60 degrees
        min_yaw = initial_yaw - 3.14/3;
        drone->log("Yaw (in, min, max): " + std::to_string(initial_yaw) + 
                   std::to_string(min_yaw) + std::to_string(max_yaw));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        yaw = (drone->getOrientation())[2];

        if (yaw >= max_yaw) {
            clockwise = false;
        } else if (yaw <= min_yaw) {
            clockwise = true;
        }

        gestures = drone->getHandGestures();
        
        if (!gestures.empty() && gestures[0] == "Open_Palm") {
            return "HAND FOUND";
        }

        drone->setLocalVelocity(0.0, 0.0, 0.0, 
                                clockwise ? yaw_speed : -yaw_speed);

        return "";
    }
    
private:
    bool clockwise = true;
    Drone* drone;
    float initial_yaw, min_yaw, max_yaw, yaw, yaw_speed;
    std::vector<std::string> gestures;
};