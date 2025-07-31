#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "vision_fase1.hpp"


class InitialTakeoffState : public fsm::State {
public:
    InitialTakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: INITIAL TAKEOFF");

        float home_x = *blackboard.get<float>("fictual_home_x");
        float home_y = *blackboard.get<float>("fictual_home_y");
        float home_z = *blackboard.get<float>("fictual_home_z");
        const Eigen::Vector3d fictual_home = Eigen::Vector3d({home_x, home_y, home_z});
        blackboard.set<Eigen::Vector3d>("home_position", fictual_home);
        
        drone->toOffboardSync();
        drone->armSync();
        drone->setHomePosition(fictual_home);


        std::vector<Base> bases;
        bases.push_back({drone->getLocalPosition(), true});
        blackboard.set<std::vector<Base>>("bases", bases);


        this->max_velocity = *blackboard.get<float>("max_vertical_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        this->print_counter = 0;

        this->pos = drone->getLocalPosition();
        this->initial_yaw = drone->getOrientation()[2];
        this->goal = Eigen::Vector3d({this->pos[0], this->pos[1], takeoff_height});


        drone->log("Initial Yaw: " + std::to_string(initial_yaw));
        drone->log("Home at: " + std::to_string(pos[0])
                    + " " + std::to_string(pos[1]) + " " + std::to_string(pos[2]));

    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        if (this->print_counter%10==0){
            drone->log("Pos: {" + std::to_string(this->pos[0]) + ", " 
            + std::to_string(this->pos[1]) + ", " + std::to_string(this->pos[2]) + "}");
        }
        this->print_counter++;
        
        
        this->pos = drone->getLocalPosition();
        Eigen::Vector3d diff = this->goal - this->pos;

        if (diff.norm() < this->position_tolerance) {
            return "INITIAL TAKEOFF COMPLETED";
        }

        Eigen::Vector3d little_goal = pos + (diff.norm() > max_velocity ?
                                            diff.normalized() * max_velocity : diff);
        
        drone->setLocalPosition(
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
};