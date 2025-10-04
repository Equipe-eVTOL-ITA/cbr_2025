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
    AlignState() : fsm::State(), x_pid(0,0,0,0,0.05), y_pid(0,0,0,0,0.05) {
        this->drone = nullptr;
        this->vision = nullptr;
        this->offset = Eigen::Vector2d::Zero();
    }

    virtual ~AlignState() = default;

    /**
     * Inicialização comum para todos os estados de alinhamento.
     * Carrega parâmetros do blackboard e configura controladores PID.
     */
    void on_enter(fsm::Blackboard &bb) override {
        this->drone = *bb.get<std::shared_ptr<Drone>>("drone");
        this->vision = *bb.get<std::shared_ptr<VisionNode>>("vision");

        if(!this->drone || !this->vision) return;

        this->drone->log("STATE: ALIGN - " + getAlignmentType());

        // Carregar parâmetros do blackboard
        loadParameters(bb);
        
        // Inicializar variáveis auxiliares
        initializeCounters();
        
        // Configurar controladores PID
        setupPidControllers();
        
        // Configurar offset específico do tipo de objeto
        configureOffset(bb);
        
        this->drone->log("Alignment configured with offset: [" + 
                        std::to_string(this->offset.x()) + ", " + 
                        std::to_string(this->offset.y()) + "]");
    }

    /**
     * Método virtual puro que deve ser implementado por cada estado derivado.
     * Implementa a lógica específica de alinhamento para cada tipo de objeto.
     */
    virtual std::string act(fsm::Blackboard &bb) = 0;

protected:
    // === Membros protegidos para acesso das classes derivadas ===
    
    // Objetos principais
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;

    // Atributos físicos
    float align_tolerance;
    float max_velocity;
    float align_descent_velocity;
    float initial_yaw;
    Eigen::Vector3d pos;
    Eigen::Vector3d orientation;
    Eigen::Vector3d approx_target; // pode ser base ou package

    // Variáveis PID
    float kp, ki, kd, setpoint;
    PidController x_pid, y_pid;

    // Variáveis auxiliares
    int print_counter;
    int no_detection_counter;
    int aligned_counter;
    int total_detected;
    int total_undetected;
    float horizontal_distance;

    // Timeout
    float detection_timeout;
    
    // Offset específico para cada tipo de objeto
    Eigen::Vector2d offset;

    // === Métodos protegidos para uso das classes derivadas ===
    
    /**
     * Configura o offset específico para o tipo de objeto.
     * Deve ser sobrescrito pelas classes derivadas.
     */
    virtual void configureOffset(fsm::Blackboard &bb) = 0;
    
    /**
     * Retorna o tipo de alinhamento para logs.
     * Deve ser sobrescrito pelas classes derivadas.
     */
    virtual std::string getAlignmentType() const = 0;
    
    /**
     * Calcula a posição do target considerando o offset configurado.
     */
    Eigen::Vector2d calculateTargetWithOffset(const Eigen::Vector3d& detected_target) {
        Eigen::Vector2d target_2d = detected_target.head<2>();
        return target_2d + this->offset;
    }
    
    /**
     * Aplica os comandos de velocidade considerando limites máximos.
     */
    void applyVelocityCommands(float x_rate, float y_rate, float z_rate = 0.0f, float yaw_rate = 0.0f) {
        // Limitar velocidade horizontal ao máximo configurado
        Eigen::Vector2d horizontal_rate(x_rate, y_rate);
        if (horizontal_rate.norm() > this->max_velocity) {
            horizontal_rate = horizontal_rate.normalized() * this->max_velocity;
        }
        
        this->drone->setLocalVelocity(horizontal_rate.x(), horizontal_rate.y(), z_rate, yaw_rate);
    }
    
    /**
     * Verifica se o alinhamento está dentro da tolerância.
     */
    bool isAligned(const Eigen::Vector2d& error) {
        return error.norm() < this->align_tolerance;
    }
    
    /**
     * Atualiza contadores e logs de debug.
     */
    void updateDebugInfo() {
        if (this->print_counter % 5 == 0) {
            this->drone->log("Position: [" + std::to_string(this->pos.x()) + ", " + 
                           std::to_string(this->pos.y()) + ", " + std::to_string(this->pos.z()) + "]");
            this->drone->log("Detected: " + std::to_string(this->total_detected) + 
                           ", Undetected: " + std::to_string(this->total_undetected));
            this->drone->log("Alignment error: " + std::to_string(this->horizontal_distance) + "m");
        }
    }

private:
    /**
     * Carrega parâmetros do blackboard.
     */
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
    
    /**
     * Inicializa contadores e variáveis auxiliares.
     */
    void initializeCounters() {
        this->print_counter = 0;
        this->no_detection_counter = 0;
        this->aligned_counter = 0;
        this->horizontal_distance = 0;
        this->total_detected = 0;
        this->total_undetected = 0;
    }
    
    /**
     * Configura controladores PID.
     */
    void setupPidControllers() {
        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
    }
};