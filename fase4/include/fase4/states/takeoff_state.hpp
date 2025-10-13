#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase4/aux/movement.hpp"

class TakeoffState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    
    float max_velocity;
    float position_tolerance;
    float initial_yaw;
    
    bool take_off_taken;

public:
    TakeoffState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        
        if(this->drone == nullptr) return;
        
        this->drone->log("STATE: TAKEOFF");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        
        auto takeoff_tolerance_ptr = bb.get<float>("takeoff_tolerance");
        this->position_tolerance = takeoff_tolerance_ptr ? *takeoff_tolerance_ptr : 0.3f; // Default: 30cm

        float takeoff_height = *bb.get<float>("takeoff_height");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});

        this->take_off_taken = *bb.get<bool>("initial_takeoff_taken");
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        this->pos = this->drone->getLocalPosition();
        
        // Recalcular goal baseado na posição atual para evitar problemas após pouso
        float takeoff_height = *bb.get<float>("takeoff_height");
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});
        
        Eigen::Vector3d diff = this->goal - this->pos;

        if(diff.norm() < this->position_tolerance) {
            if(!this->take_off_taken) {
                bb.set<bool>("initial_takeoff_taken", true);
                this->drone->log("Initial takeoff taken. Next step is to enter the house!");
                return "ENTER HOUSE";
            }
 
            this->drone->log("I've takenoff sucessfully!");
            return "TAKENOFF";
        }

        move_local_by_waypoint(this->drone, this->goal, this->max_velocity);

        return "";
    }

    void on_exit(fsm::Blackboard &bb) override {
        (void) bb;
        this->drone->log("Takeoff state completed!");
    }
};