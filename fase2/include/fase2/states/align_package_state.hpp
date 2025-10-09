#pragma once

#include <cmath>
#include <algorithm>
#include "align_state.hpp"
#include "fase2/aux/movement.hpp"

// considerando que o drone gira no sentido anti-horário para ângulos positivos

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
    
    // configurar o offset entre a garra e o centro do drone
    void configureOffset(fsm::Blackboard &bb) override {
        float package_offset_x = 0.0f; // metros - frente/trás
        float package_offset_y = 0.0f; // metros - esquerda/direita
        
        try {
            package_offset_x = *bb.get<float>("package_align_offset_x");
            package_offset_y = *bb.get<float>("package_align_offset_y");
        } catch (...) {
            // usando offsets padrão se não encontrar os outros
            this->drone->log("Using default package alignment offsets");
        }
        
        this->offset = Eigen::Vector2d(package_offset_x, package_offset_y);
        
        this->mean_package_height = *bb.get<float>("mean_package_height");
        
        this->yaw_align_tolerance = *bb.get<float>("yaw_align_tolerance");
        this->max_yaw_rate = *bb.get<float>("max_yaw_rate");
        
        // constantes PID no yaw
        float yaw_kp = *bb.get<float>("pid_yaw_kp");
        float yaw_ki = *bb.get<float>("pid_yaw_ki");
        float yaw_kd = *bb.get<float>("pid_yaw_kd");
        this->yaw_pid = PidController(yaw_kp, yaw_ki, yaw_kd, 0.0f);
    
        this->rough_tolerance = *bb.get<float>("rough_align_tolerance");
        this->movement_speed = *bb.get<float>("rough_movement_speed");

        this->fine_tolerance = *bb.get<float>("align_package_tolerance");
        this->package_base_dist_tolerance = *bb.get<float>("package_base_dist_tolerance");
    }

    // tipo de alinhamento para logs
    std::string getAlignmentType() const override {
        return "PACKAGE";
    }

public:

    void on_enter(fsm::Blackboard &bb) override {
        AlignState::on_enter(bb);
        
        this->current_phase = AlignmentPhase::ROUGH_POSITION;
        this->aligned_counter = 0;
        this->yaw_pid.reset();

        this->rough_tolerance = 0.3f; // Tolerância maior para alinhamento grosso
        this->movement_speed = 0.5f;  // Velocidade para movimento grosso
    }

    std::string act(fsm::Blackboard &bb) override {
        
        AlignState::act(bb); // ja atualiza pos e orientacao

        float current_yaw = this->orientation[2];

        // timeout
        if (this->vision->lastPackageDetectionTime() > this->detection_timeout) {
            this->drone->log("PACKAGE DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST PACKAGE";
        }

        // o pacote sempre estará sobre uma base
        if (this->vision->isTherePackageDetection() && this->vision->isThereBaseDetection()) {
            this->total_detected++;
            this->no_detection_counter = 0;

            auto package_bbox = this->vision->getClosestPackageBbox();
            auto base_bbox = this->vision->getClosestBaseBbox();

            float distance = distance_between_bboxes(package_bbox, base_bbox);
            if(distance > package_base_dist_tolerance){
                this->package_far_from_base_counter++;

                if(this->package_far_from_base_counter > 5){
                    this->drone->log("Package far from base. Distance: " + std::to_string(distance));
                    
                    // subindo um pouco para tentar achar a base
                    Eigen::Vector3d new_pos = this->pos;
                    new_pos.z() += 0.2f; // sobe 20 cm
                    move_local_by_waypoint(this->drone, new_pos, 0.1f);
                    
                    return "";
                }
                return "";
            } else {
                this->package_far_from_base_counter = 0;
            }
            
            // calcula a posição baseado apenas na proporção da tela
            this->approx_target = calculateSimpleTargetPosition(package_bbox);

            return executeSequentialAlignment(package_bbox);

        } else {
            this->total_undetected++;
            this->no_detection_counter++;
            
            // isso pode proteger de perder o package quando estiver rotacionando
            if (this->no_detection_counter > 3) {
                this->drone->log("No package detection found. Returning to initial yaw.");
                this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->initial_yaw);
                return "";
            }
        }

        return "";
    }

private:

    float mean_package_height = 0.06f;
    float hook_offset = 0.1f;
    float package_yaw_factor = 0.5f;
    float yaw_align_tolerance = 0.1f;
    float max_yaw_rate = 1.0f;

    float rough_tolerance = 0.3f; // Tolerância maior para alinhamento grosso
    float movement_speed = 0.5f;  // Velocidade para movimento grosso

    float fine_tolerance;
    float package_base_dist_tolerance;

    int package_far_from_base_counter = 0;

    // estados do alinhamento sequencial
    enum class AlignmentPhase {
        ROUGH_POSITION,     // Fase 1: Alinhamento de posição (X,Y) com o centro
        YAW_ALIGNMENT,      // Fase 2: Alinhamento de rotação (Yaw)
        FINE_POSITION       // Fase 3: Alinhamento fino com offset
    };
    
    AlignmentPhase current_phase = AlignmentPhase::ROUGH_POSITION;
    PidController yaw_pid;

    std::string executeSequentialAlignment(const BoundingBox& package_bbox) {
        switch (this->current_phase) {
            case AlignmentPhase::ROUGH_POSITION:
                return executeRoughPositionPhase();
            case AlignmentPhase::YAW_ALIGNMENT:
                return executeYawAlignmentPhase(package_bbox);
            case AlignmentPhase::FINE_POSITION:
                return executeFinePositionPhase();
            default:
                return "";
        }
    }

    std::string executeRoughPositionPhase() {
        // Fase 1: Alinhamento em X e Y (sem considerar offset)
        if (executeAlignmentPosition(this->rough_tolerance, this->movement_speed)) {
            this->current_phase = AlignmentPhase::YAW_ALIGNMENT;
            this->drone->log("Phase transition: ROUGH_POSITION -> YAW_ALIGNMENT");
            return "";
        }
        
        return "";
    }

    std::string executeYawAlignmentPhase(const BoundingBox& package_bbox) {
        // Fase 2: Alinhamento do yaw (rotação)
        
        float desired_yaw = calculateDesiredYaw(package_bbox);
        float current_yaw = this->orientation[2];
        
        // Calcular erro angular
        float yaw_error = desired_yaw - current_yaw;
        
        // Normalizar erro para [-π, π]
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        
        if (executeAlignmentYaw(desired_yaw, this->yaw_align_tolerance)) {
            this->current_phase = AlignmentPhase::FINE_POSITION;
            this->drone->log("Phase transition: YAW_ALIGNMENT -> FINE_POSITION");
            return "";
        }

        return "";
    }

    std::string executeFinePositionPhase() {
        // Fase 3: Alinhamento fino considerando offset da garra
        Eigen::Vector2d target_with_offset = calculateTargetWithOffset(this->approx_target);
        Eigen::Vector2d current_pos_2d = this->pos.head<2>();
        Eigen::Vector2d fine_error = target_with_offset - current_pos_2d;

        if (fine_error.norm() < this->align_tolerance) {
            this->aligned_counter++;
            if (this->aligned_counter > 10) {
                return "ALIGNED";
            }
        } else {
            this->aligned_counter = 0;
        }

        // Usar movimento local por waypoint para alinhamento fino
        Eigen::Vector3d fine_target_pos(target_with_offset.x(), target_with_offset.y(), this->pos.z());
        move_local_by_waypoint(this->drone, fine_target_pos, this->fine_tolerance); // Velocidade menor para precisão

        return "";
    }

    std::string getPhaseString(AlignmentPhase phase) const {
        switch (phase) {
            case AlignmentPhase::ROUGH_POSITION: return "ROUGH_POSITION";
            case AlignmentPhase::YAW_ALIGNMENT: return "YAW_ALIGNMENT";
            case AlignmentPhase::FINE_POSITION: return "FINE_POSITION";
            default: return "UNKNOWN";
        }
    }

    float calculateDesiredYaw(const BoundingBox& package_bbox) {
        float package_direction = package_bbox.rotation;
        
        // NORMALIZAR o ângulo do package primeiro
        while (package_direction > M_PI) package_direction -= 2.0 * M_PI;
        while (package_direction < -M_PI) package_direction += 2.0 * M_PI;
        
        // Testar diferentes transformações
        float option1 = this->initial_yaw + package_direction;
        float option2 = this->initial_yaw - package_direction;  
        float option3 = this->initial_yaw + package_direction + M_PI/2;
        float option4 = this->initial_yaw + package_direction - M_PI/2;
        
        // Normalizar todas as opções
        auto normalize_angle = [](float angle) {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        };
        
        option1 = normalize_angle(option1);
        option2 = normalize_angle(option2);
        option3 = normalize_angle(option3);
        option4 = normalize_angle(option4);
        
        // Escolher a opção com menor distância angular do yaw atual
        float current_yaw = this->orientation[2];
        std::vector<float> options = {option1, option2, option3, option4};
        std::vector<std::string> option_names = {"+", "-", "+90°", "-90°"};
        
        float min_error = std::abs(normalize_angle(option1 - current_yaw));
        int best_option = 0;
        float desired_yaw = option1;
        
        for (int i = 1; i < 4; i++) {
            float error = std::abs(normalize_angle(options[i] - current_yaw));
            if (error < min_error) {
                min_error = error;
                best_option = i;
                desired_yaw = options[i];
            }
        }
        
        // Log detalhado para debug
        this->drone->log("=== YAW DEBUG ===");
        this->drone->log("Package rotation (raw): " + std::to_string(package_bbox.rotation * 180.0 / M_PI) + "°");
        this->drone->log("Package direction (normalized): " + std::to_string(package_direction * 180.0 / M_PI) + "°");
        this->drone->log("Initial yaw: " + std::to_string(this->initial_yaw * 180.0 / M_PI) + "°");
        this->drone->log("Current yaw: " + std::to_string(this->orientation[2] * 180.0 / M_PI) + "°");
        this->drone->log("Option 1 (+): " + std::to_string(option1 * 180.0 / M_PI) + "°");
        this->drone->log("Option 2 (-): " + std::to_string(option2 * 180.0 / M_PI) + "°");
        this->drone->log("Option 3 (+90°): " + std::to_string(option3 * 180.0 / M_PI) + "°");
        this->drone->log("Option 4 (-90°): " + std::to_string(option4 * 180.0 / M_PI) + "°");
        this->drone->log("Best option: " + option_names[best_option] + " (error: " + std::to_string(min_error * 180.0 / M_PI) + "°)");
        this->drone->log("Selected desired_yaw: " + std::to_string(desired_yaw * 180.0 / M_PI) + "°");
        this->drone->log("================");
        
        return desired_yaw;
    }
};
