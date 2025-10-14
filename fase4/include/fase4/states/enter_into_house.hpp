#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/aux/ArenaPoint.hpp"
#include "fase4/aux/movement.hpp"

class EnterIntoHouseState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<std::vector<ArenaPoint>> checkpoints;
    ArenaPoint first_checkpoint;
    ArenaPoint front_of_the_window;
    bool yaw_aligned_with_entrance;

public:
    EnterIntoHouseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->yaw_aligned_with_entrance = false;

        this->checkpoints = *bb.get<std::shared_ptr<std::vector<ArenaPoint>>>("checkpoints");
        if(this->checkpoints == nullptr){
            this->drone->log("Checkpoints é um nullpointer");
            return;
        }

        this->first_checkpoint = this->checkpoints->at(0);

        this->front_of_the_window = *bb.get<ArenaPoint>("front_of_the_window");

        this->drone->log("STATE: ENTER INTO HOUSE");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        // ficando em frente à primeira janela (a janela de entrada para a casinha)
        if(move_local_by_waypoint(this->drone, this->front_of_the_window.coordinates, 0.5f, 0.1f) == false)
            return "";

        if(this->yaw_aligned_with_entrance)
            return "ALIGN WITH WINDOW";

        // o primeiro checkpoint está dentro da caixa, atrás da janela a ser transpassada
        this->yaw_aligned_with_entrance = lookToPoint(this->drone, this->first_checkpoint.coordinates);

        return "";
    }

};