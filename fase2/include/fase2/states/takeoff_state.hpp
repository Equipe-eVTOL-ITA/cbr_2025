#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/aux/movement.hpp"

class TakeoffState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;

    Eigen::Vector3d pos;
    Eigen::Vector3d goal;
    
    float max_velocity;
    float position_tolerance;
    float initial_yaw;
    
    bool take_off_taken;
    bool deliver_after_takeoff;

public:
    TakeoffState() : fsm::State() {}



    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        
        if(this->drone == nullptr) return;
        
        this->drone->log("STATE: TAKEOFF");

        this->max_velocity = *bb.get<float>("max_vertical_velocity");
        this->position_tolerance = *bb.get<float>("position_tolerance");

        float takeoff_height = *bb.get<float>("takeoff_height");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});

        this->take_off_taken = *bb.get<bool>("initial_takeoff_taken");
        this->deliver_after_takeoff = *bb.get<bool>("deliver_after_takeoff");
    }



    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        this->pos = this->drone->getLocalPosition();
        
        // Recalcular goal baseado na posição atual para evitar problemas após pouso
        float takeoff_height = *bb.get<float>("takeoff_height");
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});
        
        Eigen::Vector3d diff = this->goal - this->pos;
        
        this->drone->log("TAKEOFF: pos=[" + std::to_string(this->pos[0]) + "," + 
                        std::to_string(this->pos[1]) + "," + std::to_string(this->pos[2]) + 
                        "], goal=[" + std::to_string(this->goal[0]) + "," + 
                        std::to_string(this->goal[1]) + "," + std::to_string(this->goal[2]) + 
                        "], diff_norm=" + std::to_string(diff.norm()));

        if(diff.norm() < this->position_tolerance) {
            if(!this->take_off_taken) {
                bb.set<bool>("initial_takeoff_taken", true);
                return "NEXT BASE"; // proximo é comecar a procurar a pista para depois segui-la
            }
 
            // inverte o valor da booleana
            bb.set<bool>("deliver_after_takeoff", !this->deliver_after_takeoff);
            
            if(this->deliver_after_takeoff)
                return "DELIVER PACKAGE";

            return "NEXT BASE";
        }

        move_local_by_waypoint(this->drone, this->goal, this->max_velocity);

        return "";
    }
};