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
    std::string package_state;

public:
    std::string act(fsm::Blackboard &bb) override {

        AlignState::act(bb); // já atualiza pos e orientacao
        
        float current_yaw = this->orientation[2];
        
        // debug periódico
        if (this->print_counter % 5 == 0) {
            this->drone->log("Base alignment - yaw=" + std::to_string(current_yaw));
            updateDebugInfo();
        }

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

            // Calcular target com offset para alinhamento correto
            Eigen::Vector2d target_with_offset = calculateTargetWithOffset(this->approx_target);
            Eigen::Vector2d current_pos_2d = this->pos.head<2>();
            Eigen::Vector2d error = target_with_offset - current_pos_2d;
            this->horizontal_distance = error.norm();

            // Debug detalhado da detecção da base
            if (this->print_counter % 5 == 0) {
                this->drone->log("Base Detection - target pos: [" + 
                               std::to_string(this->approx_target.x()) + ", " + 
                               std::to_string(this->approx_target.y()) + ", " + 
                               std::to_string(this->approx_target.z()) + "]");
                this->drone->log("Target with offset: [" + 
                               std::to_string(target_with_offset.x()) + ", " + 
                               std::to_string(target_with_offset.y()) + "]");
                this->drone->log("Drone pos: [" + std::to_string(this->pos.x()) + 
                               ", " + std::to_string(this->pos.y()) + ", " + 
                               std::to_string(this->pos.z()) + "], orientation: " +
                               std::to_string(this->orientation[2] * 180.0 / M_PI) + "°");
                this->drone->log("Horizontal error: " + std::to_string(this->horizontal_distance) + "m");
            }

            // Verificar se está alinhado
            if (this->horizontal_distance < this->align_tolerance) {
                this->aligned_counter++;
                if (this->aligned_counter > 10) {
                    this->drone->log("Base alignment achieved - READY TO LAND");
                    return "ALIGNED";
                }
            } else {
                this->aligned_counter = 0;
                
                // Usar executeAlignmentPosition da classe base com waypoints
                // Criar target 3D com a altura atual do drone
                Eigen::Vector3d target_3d(target_with_offset.x(), target_with_offset.y(), this->pos.z());
                
                executeAlignmentPosition(target_3d);
                
                if (this->print_counter % 10 == 0) {
                    this->drone->log("Moving toward base via waypoint - error: " + std::to_string(this->horizontal_distance) + "m");
                }
            }

            // Publicar detecção da base para telemetria
            this->vision->publishBaseDetection("detected_base", this->approx_target, this->mean_base_height);

        } else {
            // sem detecção
            this->total_undetected++;
            this->no_detection_counter++;
            this->horizontal_distance = 0.0f; // Sem detecção, sem cálculo de erro
            
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
        this->package_state = *bb.get<std::string>("package_state");
        
        if (this->package_state == "get_package") {
            this->drone->log("Aligning to package pickup base");
        } else if (this->package_state == "deliver_package") {
            this->drone->log("Aligning to package delivery base");
        } else {
            this->drone->log("Aligning to base - state: " + this->package_state);
        }
    }
    
    std::string getAlignmentType() const override {
        return "BASE";
    }
};