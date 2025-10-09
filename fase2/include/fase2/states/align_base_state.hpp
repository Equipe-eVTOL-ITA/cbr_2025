#pragma once

#include "align_state.hpp"
#include "fase2/aux/movement.hpp"

/**
 * Estado de alinhamento específico para bases (landing pads).
 * 
 * Herda de AlignState e implementa lógica específica para alinhamento com bases,
 * considerando offset específico da câmera em relação ao centro do drone.
 */
class AlignBaseState : public AlignState {
public:
    AlignBaseState() : AlignState() {}

private:
    // parâmetros específicos
    float height_to_ground;
    float mean_base_height;
    bool aligned;

public:

    void on_enter(fsm::Blackboard &bb) override {
        AlignState::on_enter(bb);
        this->aligned = false;
    }

    std::string act(fsm::Blackboard &bb) override {

        AlignState::act(bb); // já atualiza pos e orientacao
        
        // float current_yaw = this->orientation[2];

        // verificar timeout
        if (this->vision->lastBaseDetectionTime() > this->detection_timeout) {
            this->drone->log("BASE DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST BASE";
        }

        if (this->vision->isThereBaseDetection()) {
            this->total_detected++;
            this->no_detection_counter = 0;

            // Obter posição da base usando a mesma lógica do vision
            this->approx_target = this->vision->getClosestBasePosition(
                this->pos, this->orientation, this->mean_base_height, true
            );

            this->aligned = executeAlignmentPosition(this->align_tolerance);

            // Verificar se está alinhado
            if (this->aligned == false) {
                this->aligned_counter++;
                if (this->aligned_counter > 10) {
                    this->drone->log("Base alignment achieved - READY TO LAND");
                    return "ALIGNED";
                }
            } else {
                this->aligned_counter = 0;
            }

            // Publicar detecção da base para telemetria
            this->vision->publishBaseDetection("detected_base", this->approx_target, this->mean_base_height);

        } else {
            // sem detecção
            this->total_undetected++;
            this->no_detection_counter++;
            
            if (this->no_detection_counter > 3) {
                this->drone->log("No base detection found. Maintaining position.");
                // Manter posição atual usando waypoint
                this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->orientation[2]);
                return "";
            }
        }

        return "";
    }

protected:
    void configureOffset(fsm::Blackboard &bb) override {
        float base_offset_x = 0.0f; // metros
        float base_offset_y = 0.0f; // metros
        
        try {
            base_offset_x = *bb.get<float>("base_align_offset_x");
            base_offset_y = *bb.get<float>("base_align_offset_y");
        } catch (...) {
            // usar os offsets padrão se não encontrar os outros
            this->drone->log("Using default base alignment offsets");
        }
        
        this->offset = Eigen::Vector2d(base_offset_x, base_offset_y);
        
        // parâmetros específicos para bases
        this->height_to_ground = *bb.get<float>("height_to_ground");
        this->mean_base_height = *bb.get<float>("mean_base_height");
    }
    
    std::string getAlignmentType() const override {
        return "BASE";
    }
};