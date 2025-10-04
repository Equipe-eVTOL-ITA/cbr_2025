#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/states/next_waypoints.hpp"
#include "fase2/aux/vision_fase2.hpp"


class NextBaseState : public NextWaypoints {
public:
    NextBaseState() : NextWaypoints() {}

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        NextWaypoints::act(bb);

        if(this->vision->isThereDetection()) {
            this->drone->log("Base detected! Aligning...");
            return "ALIGN TO BASE";
        }

        // movendo um pouco para a esquerda
        move_local_by_speed(this->drone, -1.0f, 0.0f, 0.0f);
        
        return ""; // Continua procurando pela base
    }
};