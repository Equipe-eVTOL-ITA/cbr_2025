#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"


class ApproachPackageState : public fsm::State {
public:
    ApproachPackageState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: GO TO BASE");

        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->takeoff_height = *blackboard.get<float>("takeoff_height");

        auto approx_base = *blackboard.get<Eigen::Vector2d>("approximate_base");
        this->initial_yaw = this->drone->getOrientation()[2];

        this->goal = Eigen::Vector3d(
            approx_base.x(),
            approx_base.y(),
            this->takeoff_height
        );
    }


    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        Eigen::Vector3d pos = this->drone->getLocalPosition();
        Eigen::Vector3d diff = this->goal - pos;

        if(diff.norm() < this->position_tolerance){
            return "OVER THE BASE";
        }

        Eigen::Vector3d little_goal = pos + (diff.norm() > this->max_velocity ?
                                            diff.normalized() * this->max_velocity : diff);

        this->drone->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            this->initial_yaw
        );

        return "";        
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    std::shared_ptr<Drone> drone;
    float max_velocity;
    float position_tolerance;
    float takeoff_height;
    float initial_yaw;
    Eigen::Vector3d goal;
};