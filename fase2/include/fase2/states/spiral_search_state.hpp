#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "fase2/aux/movement.hpp"
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/aux/ArenaPoint.hpp"

class SpiralSearchState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    std::shared_ptr<std::vector<ArenaPoint>> waypoints_spiral;
    int index_current_wp_spiral;

    float velocity;

public:
    SpiralSearchState() : fsm::State() {}

    void on_enter() {
        // Initialize spiral search state
        this->index_current_wp_spiral = 0;
        this->velocity = 0.5; // Default search velocity
    }

    std::string act(fsm::Blackboard &bb) override {
        auto drone = *bb.get<std::shared_ptr<Drone>>("drone");
        auto vision = *bb.get<std::shared_ptr<VisionNode>>("vision");        
        
        if(this->vision->isThereDetection()) {
            this->drone->log("Base detected during spiral search! Aligning...");
            return "ALIGN TO BASE";
        }

        ArenaPoint target_point = next_spiral_waypoint();
        move_local_by_waypoint(
            this->drone,
            target_point.coordinates,
            this->velocity
        );

        return ""; // continua neste estado
    }

private:

    ArenaPoint next_spiral_waypoint() {
        ArenaPoint point;
        if (index_current_wp_spiral < static_cast<int>(waypoints_spiral->size()))
            point = (*waypoints_spiral)[index_current_wp_spiral++];
        else
            point = (*waypoints_spiral).back();

        this->index_current_wp_spiral = std::min(index_current_wp_spiral, static_cast<int>(waypoints_spiral->size()) - 1);
        
        // somando o vetor pos atual
        point.coordinates += drone->getLocalPosition();

        return point;
    }

};