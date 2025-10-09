#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/states/approach_state.hpp"

#include "fase2/aux/movement.hpp"

class DeliverApproachState : public ApproachState {
private:
    std::shared_ptr<Drone> drone;

    void setApproachParams(fsm::Blackboard &bb) {
        ApproachState::velocity = *bb.get<float>("deliver_approach_velocity");
        ApproachState::approach_height = *bb.get<float>("deliver_approach_height");
        ApproachState::tolerance = *bb.get<float>("deliver_approach_tolerance");
    }
};