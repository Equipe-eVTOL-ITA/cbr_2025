#pragma once

#include "align_state.hpp"

/**
 * Estado de alinhamento específico para bases (landing pads).
 * 
 * Herda de AlignState e implementa lógica específica para alinhamento com bases,
 * considerando offset específico da câmera em relação ao centro do drone.
 */
class AlignBaseState : public AlignState {
public:
    AlignBaseState() : AlignState() {}

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

public:
    std::string act(fsm::Blackboard &bb) override {

        AlignState::act(bb);
        
        float yaw = this->orientation[2]; // pos e orientação já são atualizadas pela classe base
        
        // debug periódico
        if (this->print_counter % 5 == 0) {
            this->drone->log("Base alignment - yaw=" + std::to_string(yaw));
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

            this->approx_target = this->vision->getClosestBasePosition(
                this->pos, this->orientation, this->mean_base_height, true
            );

            // calcular target considerando offset
            Eigen::Vector2d target_with_offset = calculateTargetWithOffset(this->approx_target);
            Eigen::Vector2d current_pos_2d = this->pos.head<2>();
            Eigen::Vector2d alignment_error = target_with_offset - current_pos_2d;

            this->horizontal_distance = alignment_error.norm();

            // Publicar detecção da base para telemetria
            this->vision->publishBaseDetection("detected_base", this->approx_target, this->mean_base_height);

            if (isAligned(alignment_error)) {
                this->aligned_counter++;
                if (this->aligned_counter > 10) {
                    return "ALIGNED";
                }
            } else {
                this->aligned_counter = 0;
            }

            // PID
            float x_rate = this->x_pid.compute(this->setpoint - alignment_error.x());
            float y_rate = this->y_pid.compute(this->setpoint - alignment_error.y());

            applyVelocityCommands(x_rate, y_rate);

            if (this->print_counter % 5 == 0) {
                this->drone->log("Base target: [" + std::to_string(target_with_offset.x()) + 
                               ", " + std::to_string(target_with_offset.y()) + "]");
                this->drone->log("Alignment error: [" + std::to_string(alignment_error.x()) + 
                               ", " + std::to_string(alignment_error.y()) + "]");
            }

        } else {
            // sem detecção
            this->total_undetected++;
            this->no_detection_counter++;
            
            if (this->no_detection_counter > 3) {
                this->drone->log("No base detection found. Returning to initial yaw.");
                this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->initial_yaw);
                return "";
            }
        }

        return "";
    }

private:
    // parâmetros específicos
    float height_to_ground;
    float mean_base_height;
    std::string package_state;
};