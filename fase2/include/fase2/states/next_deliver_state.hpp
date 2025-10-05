#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/states/next_waypoints.hpp"
#include "fase2/aux/vision_fase2.hpp"


class NextDeliverState : public NextWaypoints {
public:
    NextDeliverState() : NextWaypoints() {}

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        return NextWaypoints::act(bb);

        if(this->vision->isThereDetection()) {
            this->drone->log("Deliver base detected! Aligning...");
            return "ALIGN TO DELIVER BASE";
        }

        return "SPIRAL SEARCH";


    }
};