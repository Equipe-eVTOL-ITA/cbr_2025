#pragma once

#include <cmath>
#include <algorithm>
#include "align_state.hpp"

/**
 * Estado de alinhamento específico para packages.
 * 
 * Herda de AlignState e implementa lógica específica para alinhamento com packages,
 * considerando offset específico da garra em relação ao centro do drone e 
 * alinhamento rotacional para packages retangulares.
 */
class AlignPackageState : public AlignState {
public:
    AlignPackageState() : AlignState() {}

protected:
    /**
     * Configura offset específico para alinhamento com packages.
     * O offset considera a diferença entre a posição da garra e o centro do drone.
     */
    void configureOffset(fsm::Blackboard &bb) override {
        // Carregar offset específico para packages (posição da garra)
        float package_offset_x = 0.0f; // metros - frente/trás
        float package_offset_y = 0.0f; // metros - esquerda/direita
        
        try {
            package_offset_x = *bb.get<float>("package_align_offset_x");
            package_offset_y = *bb.get<float>("package_align_offset_y");
        } catch (...) {
            // Se não encontrar os parâmetros específicos, usar offsets padrão da garra
            this->drone->log("Using default package alignment offsets");
        }
        
        this->offset = Eigen::Vector2d(package_offset_x, package_offset_y);
        
        // Carregar parâmetros específicos para packages
        this->mean_package_height = *bb.get<float>("mean_package_height");
        this->yaw_align_tolerance = *bb.get<float>("yaw_align_tolerance");
        this->max_yaw_rate = *bb.get<float>("max_yaw_rate");
        
        // Configurar PID para controle de yaw
        float yaw_kp = *bb.get<float>("pid_yaw_kp");
        float yaw_ki = *bb.get<float>("pid_yaw_ki");
        float yaw_kd = *bb.get<float>("pid_yaw_kd");
        this->yaw_pid = PidController(yaw_kp, yaw_ki, yaw_kd, 0.0f);
        
        this->drone->log("Package alignment configured with grappler offset: [" + 
                        std::to_string(this->offset.x()) + ", " + 
                        std::to_string(this->offset.y()) + "]");
    }
    
    /**
     * Retorna o tipo de alinhamento para logs.
     */
    std::string getAlignmentType() const override {
        return "PACKAGE";
    }

public:
    /**
     * Implementa lógica específica de alinhamento com packages.
     * Inclui alinhamento rotacional para packages retangulares.
     */
    std::string act(fsm::Blackboard &bb) override {
        (void) bb;
        this->print_counter++;

        // Atualizar posição e orientação do drone
        this->pos = this->drone->getLocalPosition();
        this->orientation = this->drone->getOrientation();
        float current_yaw = this->orientation[2];
        
        // Debug info periódico
        if (this->print_counter % 5 == 0) {
            this->drone->log("Package alignment - yaw=" + std::to_string(current_yaw));
            updateDebugInfo();
        }

        // Verificar timeout de detecção
        if (this->vision->lastPackageDetectionTime() > this->detection_timeout) {
            this->drone->log("PACKAGE DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST PACKAGE";
        }

        // Verificar se há detecção de package
        if (this->vision->isTherePackageDetection()) {
            this->total_detected++;
            this->no_detection_counter = 0;

            // Obter posição do package mais próximo usando VisionNode
            this->approx_target = this->vision->getClosestPackagePosition(
                this->pos, this->orientation, this->mean_package_height, true
            );

            // Obter bbox do package para análise de orientação
            auto package_bbox = this->vision->getClosestPackageBbox();

            // Calcular target considerando offset da garra
            Eigen::Vector2d target_with_offset = calculateTargetWithOffset(this->approx_target);
            Eigen::Vector2d current_pos_2d = this->pos.head<2>();
            Eigen::Vector2d alignment_error = target_with_offset - current_pos_2d;

            this->horizontal_distance = alignment_error.norm();

            // Calcular erro de yaw para alinhamento rotacional
            float desired_yaw = calculateDesiredYaw(package_bbox);
            float yaw_error = normalizeYawError(desired_yaw - current_yaw);

            // Verificar se está alinhado (posição e rotação)
            bool position_aligned = isAligned(alignment_error);
            bool rotation_aligned = std::abs(yaw_error) < this->yaw_align_tolerance;

            if (position_aligned && rotation_aligned) {
                this->aligned_counter++;
                if (this->aligned_counter > 10) {
                    return "PRECISELY ALIGNED";
                }
            } else {
                this->aligned_counter = 0;
            }

            // Calcular comandos PID para posição
            float x_rate = this->x_pid.compute(this->setpoint - alignment_error.x());
            float y_rate = this->y_pid.compute(this->setpoint - alignment_error.y());

            // Calcular comando PID para yaw
            float yaw_rate = this->yaw_pid.compute(-yaw_error);
            yaw_rate = std::clamp(yaw_rate, -this->max_yaw_rate, this->max_yaw_rate);

            // Aplicar comandos de velocidade incluindo yaw
            applyVelocityCommands(x_rate, y_rate, 0.0f, yaw_rate);

            if (this->print_counter % 5 == 0) {
                this->drone->log("Package target: [" + std::to_string(target_with_offset.x()) + 
                               ", " + std::to_string(target_with_offset.y()) + "]");
                this->drone->log("Position error: [" + std::to_string(alignment_error.x()) + 
                               ", " + std::to_string(alignment_error.y()) + "]");
                this->drone->log("Yaw error: " + std::to_string(yaw_error * 180.0 / M_PI) + " deg");
                this->drone->log("Position aligned: " + std::string(position_aligned ? "YES" : "NO") + 
                               ", Rotation aligned: " + std::string(rotation_aligned ? "YES" : "NO"));
            }

        } else {
            // Sem detecção
            this->total_undetected++;
            this->no_detection_counter++;
            
            if (this->no_detection_counter > 3) {
                this->drone->log("No package detection found. Returning to initial yaw.");
                this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->initial_yaw);
                return "";
            }
        }

        return "";
    }

private:
    // Parâmetros específicos para alinhamento com packages
    float mean_package_height;
    float yaw_align_tolerance;
    float max_yaw_rate;
    PidController yaw_pid;

    /**
     * Calcula o yaw desejado para alinhamento com package retangular.
     * Usa a rotação detectada no bbox para determinar orientação ideal.
     */
    float calculateDesiredYaw(const BoundingBox& package_bbox) {
        // A rotação do bbox está em radianos (CCW em coordenadas de imagem)
        float bbox_rotation = package_bbox.rotation;
        
        // Converter rotação da bbox para orientação desejada do drone
        // Assumindo que queremos alinhar com o eixo longitudinal do package
        float desired_yaw = this->initial_yaw + bbox_rotation;
        
        // Normalizar para [-π, π]
        while (desired_yaw > M_PI) desired_yaw -= 2.0 * M_PI;
        while (desired_yaw < -M_PI) desired_yaw += 2.0 * M_PI;
        
        return desired_yaw;
    }
    
    /**
     * Normaliza erro de yaw para [-π, π].
     */
    float normalizeYawError(float yaw_error) {
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        return yaw_error;
    }
};
