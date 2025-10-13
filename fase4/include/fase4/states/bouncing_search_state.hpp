#pragma once

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/aux/movement.hpp"
#include <Eigen/Eigen>
#include <map>

#define ALTA true
#define BAIXA false

class BouncingSearchState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    Eigen::Vector3d initial_position;
    float initial_yaw;

    // Parâmetros carregados do blackboard
    float up_height;
    float down_height;
    float middle_height;

    typedef struct BounceInfo {
        float yaw;
        float height;
    } BounceInfo;

    enum class Phase {
        UP_FRONT = 0,
        UP_RIGHT,
        DOWN_RIGHT,
        DOWN_LEFT,
        UP_LEFT,
        UP_FRONT2,
        DOWN_FRONT,
        MIDDLE_FRONT,
        END
    };

    Phase current_phase = Phase::UP_FRONT;

    // Map será inicializado no on_enter
    std::map<Phase, BounceInfo> phase_info;

    bool height_reached = false;
    bool phase_completed = false;

public:
    BouncingSearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        this->vision = *bb.get<std::shared_ptr<VisionNode>>("vision");

        if(this->drone == nullptr || this->vision == nullptr) return;

        // Carregar parâmetros do blackboard
        this->up_height = *bb.get<float>("up_height");
        this->down_height = *bb.get<float>("down_height");
        this->middle_height = *bb.get<float>("middle_height");

        // Inicializar o map com os valores carregados
        this->phase_info = {
            {Phase::UP_FRONT,     {0.0f,    this->up_height}},
            {Phase::UP_RIGHT,     {1.57f,   this->up_height}},
            {Phase::DOWN_RIGHT,   {1.57f,   this->down_height}},
            {Phase::DOWN_LEFT,    {-1.57f,  this->down_height}},
            {Phase::UP_LEFT,      {-1.57f,  this->up_height}},
            {Phase::UP_FRONT2,    {0.0f,    this->up_height}},
            {Phase::DOWN_FRONT,   {0.0f,    this->down_height}},
            {Phase::MIDDLE_FRONT, {0.0f,    this->middle_height}}
        };

        this->initial_position = this->drone->getLocalPosition();
        this->initial_yaw = this->drone->getOrientation()[2];
        
        this->height_reached = false;
        this->phase_completed = false;

        this->drone->log("STATE: BOUNCING SEARCH");
    }

    std::string act(fsm::Blackboard &bb) override {
        (void) bb;

        Eigen::Vector3d target_position = this->initial_position;
        target_position.z() = this->phase_info[this->current_phase].height;
        
        this->scan(bb);

        this->height_reached = move_local_by_waypoint(this->drone, target_position, 0.2f, 0.05f);
        if(this->height_reached){
            float target_yaw = this->initial_yaw + this->phase_info[this->current_phase].yaw;
            this->phase_completed = rotateYaw(this->drone, target_yaw, 0.3f, 0.05f);

            if(this->phase_completed){
                // avança para a próxima fase
                this->current_phase = static_cast<Phase>((static_cast<int>(this->current_phase) + 1));

                if(this->current_phase == Phase::END) {
                    if(*bb.get<bool>("espera_janela"))
                        return "LOOK TO WINDOW";

                    return "BOUNCING SEARCH COMPLETED";
                }

                this->height_reached = false;
                this->phase_completed = false;
            }

        }

        return "";
    }

private:
    

    void scan(fsm::Blackboard &bb) {
        (void) bb;
        // implementar aqui a lógica de escaneamento de QR code
        if (this->vision->isThereQRCodeDetection()) {
            QRCode qr = this->vision->getQRCode();
            this->drone->log("QR Code detected! Content: " + qr.content);
            // o python está salvando a imagem automaticamente
        }
    }

};