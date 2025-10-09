#pragma once

#include <cmath>
#include <algorithm>
#include "align_state.hpp"
#include "fase2/aux/movement.hpp"

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
        this->horizontal_distance = 0.0f;
        this->yaw_pid.reset();

        this->rough_tolerance = 0.3f; // Tolerância maior para alinhamento grosso
        this->movement_speed = 0.5f;  // Velocidade para movimento grosso
    }

    std::string act(fsm::Blackboard &bb) override {
        
        AlignState::act(bb); // ja atualiza pos e orientacao

        // float current_yaw = this->orientation[2];

        // timeout
        if (this->vision->lastPackageDetectionTime() > this->detection_timeout) {
            this->drone->log("PACKAGE DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST PACKAGE";
        }

        if (this->vision->isTherePackageDetection()) {
            this->total_detected++;
            this->no_detection_counter = 0;

            auto package_bbox = this->vision->getClosestPackageBbox();
            
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

        this->horizontal_distance = fine_error.norm();

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

    // posicao do target baseado na proporção da imagem
    Eigen::Vector3d calculateSimpleTargetPosition(const BoundingBox& package_bbox) {

        // validando a entrada
        if (!std::isfinite(package_bbox.center_x) || !std::isfinite(package_bbox.center_y) ||
            package_bbox.center_x < 0.0f || package_bbox.center_x > 1.0f ||
            package_bbox.center_y < 0.0f || package_bbox.center_y > 1.0f) {
            this->drone->log("ERROR: Invalid bbox coordinates detected!");
            return this->pos; // posição atual como fallback
        }
        
        float center_x_norm = package_bbox.center_x; // [0, 1]
        float center_y_norm = package_bbox.center_y; // [0, 1]
        
        // Converter para offset em metros baseado na altura do drone
        float drone_height = std::abs(this->pos.z()); // Altura positiva
        if (drone_height < 0.1f) drone_height = 2.5f; // Fallback para altura padrão
        float fov_scale = drone_height * 0.3f; // Ajustar escala se necessário
        
        // Calcular offset em relação ao centro da imagem (0.5, 0.5)
        // COORDENADAS DA IMAGEM: X=direita, Y=baixo
        float image_offset_x = (center_x_norm - 0.5f) * fov_scale * 2.0f; // Direita = positivo
        float image_offset_y = (center_y_norm - 0.5f) * fov_scale * 2.0f; // Baixo = positivo
        
        // Limitar offsets para evitar valores extremos
        image_offset_x = std::clamp(image_offset_x, -2.0f, 2.0f);
        image_offset_y = std::clamp(image_offset_y, -2.0f, 2.0f);
        
        // Aplicar rotação baseada no yaw atual do drone
        float current_yaw = this->orientation[2];
        
        // Mapeamento de coordenadas
        // Para câmera apontando para baixo:
        // - Y da imagem (baixo) → X do drone (frente): INVERTER (para convergir)
        // - X da imagem (direita) → Y do drone (direita): MANTER (para convergir)
        float drone_frame_x = -image_offset_y;  // Baixo na imagem = movimento PARA FRENTE
        float drone_frame_y = image_offset_x;   // Direita na imagem = movimento PARA DIREITA
        
        // Aplicar rotação do yaw para converter para coordenadas globais
        float cos_yaw = std::cos(current_yaw);
        float sin_yaw = std::sin(current_yaw);

        // Verificar se os valores trigonométricos são válidos
        if (!std::isfinite(cos_yaw) || !std::isfinite(sin_yaw)) {
            this->drone->log("ERROR: Invalid trigonometric values!");
            return this->pos; // Retornar posição atual como fallback
        }
        
        float world_offset_x = drone_frame_x * cos_yaw - drone_frame_y * sin_yaw;
        float world_offset_y = drone_frame_x * sin_yaw + drone_frame_y * cos_yaw;

        // Limitar offsets finais para evitar valores extremos
        world_offset_x = std::clamp(world_offset_x, -5.0f, 5.0f);
        world_offset_y = std::clamp(world_offset_y, -5.0f, 5.0f);
        
        // Posição final no mundo
        float world_x = this->pos.x() + world_offset_x;
        float world_y = this->pos.y() + world_offset_y;
        float world_z = 0.0f; // Package no chão

        // Verificar se o resultado é válido
        if (!std::isfinite(world_x) || !std::isfinite(world_y)) {
            this->drone->log("ERROR: Invalid world coordinates calculated!");
            return this->pos; // posição atual como fallback
        }
        
        /*/ Debug do cálculo
        if (this->print_counter % 10 == 0) {
            this->drone->log("TARGET_CALC: bbox=[" + std::to_string(center_x_norm) + 
                           "," + std::to_string(center_y_norm) + "], img_offset=[" +
                           std::to_string(image_offset_x) + "," + std::to_string(image_offset_y) + 
                           "], drone_frame=[" + std::to_string(drone_frame_x) + "," + 
                           std::to_string(drone_frame_y) + "], yaw=" + 
                           std::to_string(current_yaw * 180.0 / M_PI) + "°");
            this->drone->log("WORLD_TARGET: [" + std::to_string(world_x) + "," + 
                           std::to_string(world_y) + "] from drone_pos=[" + 
                           std::to_string(this->pos.x()) + "," + std::to_string(this->pos.y()) + "]");
        }*/
        
        return Eigen::Vector3d(world_x, world_y, world_z);
    }


    float calculateDesiredYaw(const BoundingBox& package_bbox) {
        // A rotação do bbox está em radianos (CCW em coordenadas de imagem)
        float bbox_rotation = package_bbox.rotation;
        
        // Converter rotação da bbox para orientação desejada do drone
        // Adicionando 90 graus (π/2) em relação ao alinhamento original
        float desired_yaw = this->initial_yaw + bbox_rotation + M_PI/2.0;
        
        // Normalizar para [-π, π]
        while (desired_yaw > M_PI) desired_yaw -= 2.0 * M_PI;
        while (desired_yaw < -M_PI) desired_yaw += 2.0 * M_PI;
        
        return desired_yaw;
    }
};
