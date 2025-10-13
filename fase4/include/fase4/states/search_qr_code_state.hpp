#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/aux/ArenaPoint.hpp"
#include "fase4/states/bouncing_search_state.hpp"
#include "fase4/aux/movement.hpp"

class SearchQRCodeState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::vector<ArenaPoint> checkpoints;
    std::vector<bool> checkpoints_espera_janela;
    int checkpoint_index;

    float velocity;

    bool arrived_at_checkpoint = false;

public:
    SearchQRCodeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        if(this->drone == nullptr) return;

        this->velocity = *bb.get<float>("search_velocity");

        this->checkpoint_index = static_cast<int>(*bb.get<float>("checkpoint_index"));
        this->checkpoints = *bb.get<std::vector<ArenaPoint>>("checkpoints");
        this->checkpoints_espera_janela = *bb.get<std::vector<bool>>("checkpoints_espera_janela");

        this->arrived_at_checkpoint = false;

        this->drone->log("STATE: SEARCH QR CODE");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        if (static_cast<size_t>(this->checkpoint_index) >= this->checkpoints.size()) {
            this->drone->log("All checkpoints visited. Leaving the house.");
            return "RETURN HOME";
        }
        
        this->arrived_at_checkpoint = move_local_by_waypoint(
            this->drone, 
            this->checkpoints[this->checkpoint_index].coordinates, 
            this->velocity, 0.1f
        );

        if(this->arrived_at_checkpoint){
            bool espera_janela = this->checkpoints_espera_janela[this->checkpoint_index];
            bb.set<bool>("espera_janela", espera_janela);

            if(static_cast<size_t>(this->checkpoint_index+1) < this->checkpoints.size())
                bb.set<Eigen::Vector3d>("window_position", this->checkpoints[this->checkpoint_index+1].coordinates);

            return "BOUNCING SEARCH";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &bb) override {
        // Incrementa o índice do checkpoint para a próxima vez
        bb.set<float>("checkpoint_index", static_cast<float>(this->checkpoint_index + 1));
    }
};