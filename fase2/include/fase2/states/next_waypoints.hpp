#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/aux/ArenaPoint.hpp"
#include "fase2/aux/PairOfArenaPoints.hpp"
#include "fase2/aux/movement.hpp"
#include "fase2/aux/vision_fase2.hpp"

class NextWaypoints : public fsm::State {
protected:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    
private:
    float max_velocity;
    bool isBase;

    PairOfArenaPoints current_pair;
    ArenaPoint target_point;

public:
    NextWaypoints() : fsm::State(), current_pair(ArenaPoint(), ArenaPoint()), target_point() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        this->vision = *bb.get<std::shared_ptr<VisionNode>>("vision");
        
        if(this->drone == nullptr) return;
        if(this->vision == nullptr) return;

        this->drone->log("STATE: NEXT BASE");

        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
    
        // Fix: Use float instead of int for current_waypoint_index
        float index_float = *bb.get<float>("current_waypoint_index");
        int index = static_cast<int>(index_float);
        
        auto pairs_ptr = bb.get<std::shared_ptr<std::vector<PairOfArenaPoints>>>("waypoint_pairs");
        if(!pairs_ptr || !*pairs_ptr) return;
        
        std::vector<PairOfArenaPoints>& pairs = **pairs_ptr;

        this->drone->log("Tamanho dos pares de waypoints: " + std::to_string(pairs.size()));
        this->drone->log("Índice atual do waypoint: " + std::to_string(index));

        if(index >= static_cast<int>(pairs.size())){
            // acabaram as base
            bb.set<bool>("are_there_packages_yet", false);
            return;
        }

        this->isBase = (index % 2 == 0); // é base se o índice for par

        this->current_pair = pairs[index];
    }
    
    virtual std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        if(*bb.get<bool>("are_there_packages_yet") == false)
            return "NO MORE BASES";

        this->target_point = this->current_pair.base;

        bool arrived = move_local_by_waypoint(
            this->drone,
            this->target_point.coordinates,
            this->max_velocity
        );

        if(!arrived) return ""; // ainda não chegou no waypoint
        
        return "ARRIVED"; // Add default return for when arrived
    }

    void on_exit(fsm::Blackboard &bb) override {
        (void) bb;
    
        // atualizando o índice do próximo waypoint
        float current_index = *bb.get<float>("current_waypoint_index");
        bb.set<float>("current_waypoint_index", current_index + 1.0f);
    }
};