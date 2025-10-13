#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/aux/movement.hpp"

class ThroughWindowState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

    float pass_through_speed;
    float pass_through_distance;
    float pass_through_tolerance = 0.1f;
    float pass_through_timeout = 10.0f;

    bool passed_through;

public:
    ThroughWindowState() : fsm::State(), passed_through(false) {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->pass_through_speed = *bb.get<float>("pass_through_speed");
        this->pass_through_distance = *bb.get<float>("pass_through_distance");
        this->pass_through_tolerance = *bb.get<float>("pass_through_tolerance");
        this->pass_through_timeout = *bb.get<float>("pass_through_timeout");
        this->passed_through = false;

        this->drone->log("STATE: THROUGH WINDOW");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        this->passed_through = move_local_by_sentido(this->drone, Sentido::FRENTE, this->pass_through_distance, this->pass_through_speed, this->pass_through_tolerance, true, this->pass_through_timeout);
    
        if(this->passed_through) {
            this->drone->log("THROUGH WINDOW: Passed through the window.");
            return "PASSED";
        }

        return "";
    }

};