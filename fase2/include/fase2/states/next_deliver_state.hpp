#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/states/next_waypoints.hpp"
#include "fase2/aux/vision_fase2.hpp"


class NextDeliverState : public NextWaypoints {
public:
    NextDeliverState() : NextWaypoints() {}

    void on_enter(fsm::Blackboard &bb) override {
        NextWaypoints::on_enter(bb);
        this->drone->log("STATE: NEXT DELIVER");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        std::string std_next_waypoint = NextWaypoints::act(bb);

        if(std_next_waypoint != "ARRIVED")
            return std_next_waypoint;

        // ARRIVED at waypoint

        if(this->vision->isThereBaseDetection()) { // no deliver tem que alinhar com a base de entrega
            this->drone->log("Deliver base detected! Aligning...");
            return "ALIGN TO DELIVER";
        }

        this->drone->log("No deliver base detected! Better do a spiral search...");

        return "SPIRAL SEARCH";
    }

    void on_exit(fsm::Blackboard &bb) override {
        this->drone->log("Exiting NEXT DELIVER state.");
        // atualizando o índice do próximo waypoint
        float current_index = *bb.get<float>("current_waypoint_index");
        bb.set<float>("current_waypoint_index", current_index + 1.0f);
        this->drone->log("Next waypoint index set to " + std::to_string(current_index + 1.0f));
    }

private:

    void setTargetPoint() override {
        this->target_point = this->current_pair.deliver;
    }

};