#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/states/next_waypoints.hpp"
#include "fase2/aux/vision_fase2.hpp"


class NextBaseState : public NextWaypoints {
public:
    NextBaseState() : NextWaypoints() {}

    void on_enter(fsm::Blackboard &bb) override {
        NextWaypoints::on_enter(bb);
        this->drone->log("STATE: NEXT BASE");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        std::string std_next_waypoint = NextWaypoints::act(bb);

        if(std_next_waypoint != "ARRIVED")
            return std_next_waypoint;

        // ARRIVED at waypoint

        this->drone->log("Searching for base by moving left...");

        if(this->vision->isTherePackageDetection()) { // no base tem que alinhar com o pacote a ser pego
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
        //move_local_by_speed(this->drone, 1.0f, 0.0f, 0.0f);

        return true;
    }

    void setTargetPoint() override {
        this->target_point = this->current_pair.base;
    }
};