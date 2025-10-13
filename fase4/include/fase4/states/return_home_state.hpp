#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase4/states/next_waypoints.hpp"


class ReturnHomeState : public NextWaypoints {
public:
    ReturnHomeState() : NextWaypoints() {}

    void on_enter(fsm::Blackboard &bb) override {
        NextWaypoints::on_enter(bb);

        this->home_x = *bb.get<float>("fictual_home_x"),
        this->home_y = *bb.get<float>("fictual_home_y"),
        this->home_z = *bb.get<float>("takeoff_height"); // para evitar colisoes

        bb.set<bool>("is_returning_home", true);

        this->drone->log("STATE: RETURN HOME");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        return NextWaypoints::act(bb);
    }

    void on_exit(fsm::Blackboard &bb) override {
        this->drone->log("ARRIVED AT HOME");
        bb.set<bool>("at_home", true);
    }

private:

    float home_x;
    float home_y;
    float home_z;

    void setTargetPoint() override {
        this->target_point = Eigen::Vector3d(
            this->home_x,
            this->home_y,
            this->home_z
        );
    }

};