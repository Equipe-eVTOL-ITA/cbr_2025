#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "vision_fase2.hpp"

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) return;
        
        this->drone->log("");
        this->drone->log("STATE: TAKEOFF");


        this->max_velocity = *blackboard.get<float>("max_vertical_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        this->pos = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});

        
        bool finished_bases = *blackboard.get<bool>("finished_bases");
        this->return_statement = finished_bases ? "FINISHED BASES" : "NEXT BASE";

        if (this->drone->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            this->drone->toOffboardSync();
            this->drone->armSync();
        }
        
        this->print_counter = 0;
        // this->drone->log("Initial Yaw: " + std::to_string(initial_yaw));
        // this->drone->log("Takeoff at: " + std::to_string(pos[0])
                    // + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        if (this->print_counter%10==0){
            this->drone->log("Pos: {" + std::to_string(this->pos[0]) + ", " 
            + std::to_string(this->pos[1]) + ", " + std::to_string(this->pos[2]) + "}");
        }
        this->print_counter++;
        
        this->pos = this->drone->getLocalPosition();
        Eigen::Vector3d diff = this->goal - this->pos;

        if (diff.norm() < this->position_tolerance) {
            return this->return_statement;
        }

        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ?
                                            diff.normalized() * max_velocity : diff);
        
        this->drone->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            this->initial_yaw);
        
        return "";
    }

private:
    float max_velocity;
    Eigen::Vector3d pos, goal, goal_diff, little_goal;
    std::shared_ptr<Drone> drone;
    int print_counter;
    float initial_yaw;
    float position_tolerance;
    std::string return_statement;
};