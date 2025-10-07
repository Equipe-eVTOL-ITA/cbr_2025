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

        std::string std_next_waypoint = NextWaypoints::act(bb);
        if(std_next_waypoint != "ARRIVED")
            return std_next_waypoint;

        // ARRIVED at waypoint

        this->drone->log("Searching for base by moving left...");

        if(this->vision->isThereBaseDetection()) {
            this->drone->log("Base detected! Aligning...");
            return "ALIGN TO PACKAGE";
        }

        if(!this->search_base())
            return "NOT FOUND";

        return ""; // Continua procurando pela base
    }

private:

    bool search_base() {
        // Implementar lógica de busca pela base
        // movendo um pouco para a esquerda
        move_local_by_speed(this->drone, 1.0f, 0.0f, 0.0f);

        return true;
    }
};