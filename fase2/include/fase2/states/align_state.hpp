#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/aux/PidController.hpp"
#include "fase2/aux/movement.hpp"

/**
 * Classe base abstrata para estados de alinhamento vertical com objetos detectados.
 * 
 * Esta classe implementa a lógica comum de alinhamento e permite que classes derivadas
 * especializem o comportamento para diferentes tipos de objetos (base, package).
 * 
 * Características:
 * - Implementa on_enter() comum para todos os tipos de alinhamento
 * - Define act() como método virtual puro para implementação específica
 * - Suporte a offsets específicos para cada tipo de objeto
 * - Compatível com VisionNode que diferencia entre bases e packages
 */
class AlignState : public fsm::State {
public:
    AlignState() : fsm::State(), x_pid(0.0, 0.0, 0.0, 0.0, 0.05), y_pid(0.0, 0.0, 0.0, 0.0, 0.05) {
        this->drone = nullptr;
        this->vision = nullptr;
        this->offset = Eigen::Vector2d::Zero();
    }

    virtual ~AlignState() = default;

    // carrega parâmetros do blackboard e configura controladores PID.
    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        this->vision = *bb.get<std::shared_ptr<VisionNode>>("vision");

        if(!this->drone || !this->vision) return;

        this->drone->log("STATE: ALIGN - " + getAlignmentType());

        loadParameters(bb); // parametros da blackboard
        
        initializeCounters(); // variaveis aux
        
        setupPidControllers(); // config PID
        
        configureOffset(bb); // offset específico para cada implementação
        
        this->drone->log("Alignment configured with offset: [" + 
                        std::to_string(this->offset.x()) + ", " + 
                        std::to_string(this->offset.y()) + "]");
    }

    // implementação padrão que pode ser invocada pelas classes derivadas
    // lógica comum de alinhamento
    virtual std::string act(fsm::Blackboard &bb) {
        (void) bb;
       
        this->print_counter++; // padrão para todos os estados de alinhamento
        
        // atualizar posição e orientação do drone
        this->pos = this->drone->getLocalPosition();
        this->orientation = this->drone->getOrientation();
        
        // updateDebugInfo();
        
        return "";
    }

protected:
    
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    // atributos físicos
    float align_tolerance;
    float max_velocity;
    float align_descent_velocity;
    float initial_yaw;
    Eigen::Vector3d pos;
    Eigen::Vector3d orientation;
    Eigen::Vector3d approx_target; // pode ser base ou package

    // PID
    float kp, ki, kd, setpoint;
    PidController x_pid, y_pid;

    // variáveis aux
    int print_counter;
    int no_detection_counter;
    int aligned_counter;
    int total_detected;
    int total_undetected;

    // timeout
    float detection_timeout;
    
    // offset específico para cada tipo de objeto
    Eigen::Vector2d offset;


    // configura o offset específico para o tipo de objeto
    virtual void configureOffset(fsm::Blackboard &bb) = 0;
    
    // retorna o tipo de alinhamento
    virtual std::string getAlignmentType() const = 0;
    
    // considera o offset para calcular o target final
    Eigen::Vector2d calculateTargetWithOffset(const Eigen::Vector3d& detected_target) {
        Eigen::Vector2d target_2d = detected_target.head<2>();
        return target_2d + this->offset;
    }
    
    // aplica os comandos de velocidade considerando limites máximos
    void applyVelocityCommands(float x_rate, float y_rate, float z_rate = 0.0f, float yaw_rate = 0.0f) {
        // Limitar velocidade horizontal ao máximo configurado
        Eigen::Vector2d horizontal_rate(x_rate, y_rate);
        if (horizontal_rate.norm() > this->max_velocity) {
            horizontal_rate = horizontal_rate.normalized() * this->max_velocity;
        }
        
        this->drone->setLocalVelocity(horizontal_rate.x(), horizontal_rate.y(), z_rate, yaw_rate);
    }
    
    // verifica alinhamento com tolerancia
    bool isAligned(const Eigen::Vector2d& error) {
        return error.norm() < this->align_tolerance;
    }

public:
    // posicao do target baseado na proporção da imagem
    Eigen::Vector3d calculateSimpleTargetPosition(const BoundingBox& bbox) {

        // validando a entrada
        if (!std::isfinite(bbox.center_x) || !std::isfinite(bbox.center_y) ||
            bbox.center_x < 0.0f || bbox.center_x > 1.0f ||
            bbox.center_y < 0.0f || bbox.center_y > 1.0f) {
            this->drone->log("ERROR: Invalid bbox coordinates detected!");
            return this->pos; // posição atual como fallback
        }
        
        float center_x_norm = bbox.center_x; // [0, 1]
        float center_y_norm = bbox.center_y; // [0, 1]
        
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
        
        // Posição final no mundo (target no chão)
        float world_x = this->pos.x() + world_offset_x;
        float world_y = this->pos.y() + world_offset_y;
        float world_z = 0.0f; // Target no chão

        // Verificar se o resultado é válido
        if (!std::isfinite(world_x) || !std::isfinite(world_y)) {
            this->drone->log("ERROR: Invalid world coordinates calculated!");
            return this->pos; // posição atual como fallback
        }
        
        return Eigen::Vector3d(world_x, world_y, world_z);
    }

protected:

    bool executeAlignmentPosition(float tolerance_error, float tolerance_movement){
        //this->drone->log("Executing position alignment...");

        Eigen::Vector2d current_pos_2d = this->pos.head<2>();
        Eigen::Vector2d target = this->approx_target.head<2>();
        Eigen::Vector2d error = target - current_pos_2d;

        if(error.norm() < tolerance_error)
            return true;

        // Usar movimento local por waypoint para alinhamento grosso
        Eigen::Vector3d target_pos(target.x(), target.y(), this->pos.z());
        move_local_by_waypoint(this->drone, target_pos, tolerance_movement);

        return false;
    }

    bool executeAlignmentPosition(float tolerance){
        return this->executeAlignmentPosition(tolerance, tolerance);
    }

    bool executeAlignmentYaw(float desired_yaw, float tolerance){
        // this->drone->log("Executing yaw alignment...");

        float current_yaw = this->orientation[2];
        float yaw_error = normalizeYawError(desired_yaw - current_yaw);

        if (std::abs(yaw_error) < tolerance) {
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop rotation
            return true;
        }

        // Usar rotação específica para yaw
        rotateYaw(this->drone, desired_yaw);

        return false;
    }

    // normaliza erro de yaw para [-π, π]
    float normalizeYawError(float yaw_error) {
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
        return yaw_error;
    }

    void updateDebugInfo() {
        if (this->print_counter % 5 == 0) {
            this->drone->log("Position: [" + std::to_string(this->pos.x()) + ", " + 
                           std::to_string(this->pos.y()) + ", " + std::to_string(this->pos.z()) + "]");
            this->drone->log("Detected: " + std::to_string(this->total_detected) + 
                           ", Undetected: " + std::to_string(this->total_undetected));
        }
    }

private:
    // parâmetros do blackboard
    void loadParameters(fsm::Blackboard &bb) {
        this->detection_timeout = *bb.get<float>("detection_timeout");
        this->align_tolerance = *bb.get<float>("align_tolerance");
        this->max_velocity = *bb.get<float>("max_horizontal_velocity");
        this->align_descent_velocity = *bb.get<float>("align_descent_velocity");
        this->initial_yaw = this->drone->getOrientation()[2];
        
        this->kp = *bb.get<float>("pid_pos_kp");
        this->ki = *bb.get<float>("pid_pos_ki");
        this->kd = *bb.get<float>("pid_pos_kd");
        this->setpoint = *bb.get<float>("setpoint");
    }
    
    // contadores e var aux
    void initializeCounters() {
        this->print_counter = 0;
        this->no_detection_counter = 0;
        this->aligned_counter = 0;
        this->total_detected = 0;
        this->total_undetected = 0;
    }

    // configura controladores PID
    void setupPidControllers() {
        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
    }
};