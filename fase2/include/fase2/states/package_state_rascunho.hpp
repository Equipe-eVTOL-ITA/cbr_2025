#pragma once

#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <chrono>
#include <string>

/**
 * Estado para manipulação de packages usando a garra.
 * 
 * Funciona como toggle - pode abrir (soltar) ou fechar (pegar) a garra
 * baseado no parâmetro configurado no blackboard.
 * 
 * Operações suportadas:
 * - "pickup": Fechar garra para pegar package
 * - "drop": Abrir garra para soltar package
 */
class PackageState : public fsm::State {
public:
    PackageState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone == nullptr) return;

        // Determinar operação a ser realizada
        auto operation_ptr = blackboard.get<std::string>("package_operation");
        operation = operation_ptr ? *operation_ptr : "pickup"; // Default: pegar package
        
        // Configurar parâmetros baseados na operação
        if (operation == "pickup") {
            drone->log("STATE: PICKUP PACKAGE");
            script_command = "close";
            success_message = "PACKAGE_PICKED_UP";
            operation_description = "picking up package";
        } else if (operation == "drop") {
            drone->log("STATE: DROP PACKAGE");
            script_command = "open";
            success_message = "PACKAGE_DROPPED";
            operation_description = "dropping package";
        } else {
            drone->log("WARNING: Unknown package operation '" + operation + "', defaulting to pickup");
            operation = "pickup";
            script_command = "close";
            success_message = "PACKAGE_PICKED_UP";
            operation_description = "picking up package";
        }

        // Configurar timeout da operação
        auto timeout_ptr = blackboard.get<float>("package_operation_timeout");
        operation_timeout_ms = timeout_ptr ? static_cast<int>(*timeout_ptr * 1000) : 5000; // Default: 5 segundos

        drone->log("Starting " + operation_description + " operation");
        drone->log("Timeout configured: " + std::to_string(operation_timeout_ms) + "ms");

        start_time = std::chrono::steady_clock::now();
        command_counter = 0;
        operation_completed = false;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        
        // Enviar comando para garra periodicamente (a cada 5 iterações)
        if (command_counter % 5 == 0) {
            std::string command = "python3 /home/evtol/frtl_2025_ws/src/scripts/gancho_drop.py " + script_command;
            int ret = std::system(command.c_str());
            
            drone->log("Garra command '" + script_command + "' returned " + std::to_string(ret));
            
            // Verificar se comando foi executado com sucesso
            if (ret == 0) {
                drone->log("Garra command executed successfully");
            } else {
                drone->log("WARNING: Garra command failed with return code " + std::to_string(ret));
            }
        }
        
        // Verificar timeout da operação
        if (elapsed.count() > operation_timeout_ms) {
            drone->log("Package operation completed after " + std::to_string(elapsed.count()) + "ms");
            operation_completed = true;
            return success_message;
        }
        
        // Manter drone estável durante operação
        drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        command_counter++;
        
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        
        std::string exit_message = "Exiting " + operation_description + " state";
        if (operation_completed) {
            exit_message += " - Operation completed successfully";
        } else {
            exit_message += " - Operation interrupted";
        }
        
        drone->log(exit_message);
        
        // Salvar status da operação no blackboard para estados subsequentes
        blackboard.set("last_package_operation", operation);
        blackboard.set("package_operation_completed", operation_completed);
    }

    /**
     * Verifica se o package foi manipulado com sucesso.
     * 
     * @return true se o package foi pego/solto com sucesso, false caso contrário
     * 
     * TODO: Implementar verificação usando sensores da PixHawk
     * - Para pickup: Verificar aumento no thrust dos motores (peso adicional)
     * - Para drop: Verificar diminuição no thrust dos motores (peso removido)
     * - Usar dados disponíveis no ROS2 do sistema de controle
     */
    bool isPackageManipulated() const {
        // TODO: Implementar lógica de verificação usando thrust dos motores
        // Exemplo de implementação futura:
        //
        // if (operation == "pickup") {
        //     // Verificar se thrust aumentou indicando peso adicional
        //     float current_thrust = getTotalMotorThrust();
        //     float baseline_thrust = getBaselineThrustForCurrentAltitude();
        //     float thrust_difference = current_thrust - baseline_thrust;
        //     
        //     // Se thrust aumentou significativamente, package foi pego
        //     return thrust_difference > PACKAGE_WEIGHT_THRUST_THRESHOLD;
        // } 
        // else if (operation == "drop") {
        //     // Verificar se thrust diminuiu indicando peso removido
        //     float current_thrust = getTotalMotorThrust();
        //     float baseline_thrust = getBaselineThrustForCurrentAltitude();
        //     float thrust_difference = baseline_thrust - current_thrust;
        //     
        //     // Se thrust diminuiu significativamente, package foi solto
        //     return thrust_difference > PACKAGE_WEIGHT_THRUST_THRESHOLD;
        // }
        
        // Por enquanto, assumir sucesso baseado no tempo decorrido
        return operation_completed;
    }

    /**
     * Retorna o tipo de operação sendo realizada.
     */
    std::string getOperation() const {
        return operation;
    }

    /**
     * Retorna se a operação foi completada.
     */
    bool isOperationCompleted() const {
        return operation_completed;
    }

    /**
     * Retorna o tempo decorrido desde o início da operação em milissegundos.
     */
    long getElapsedTime() const {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time);
        return elapsed.count();
    }

private:
    std::shared_ptr<Drone> drone;
    std::chrono::steady_clock::time_point start_time;
    int command_counter;
    
    // Configuração da operação
    std::string operation;           // "pickup" ou "drop"
    std::string script_command;      // "close" ou "open"
    std::string success_message;     // Mensagem de retorno para FSM
    std::string operation_description; // Descrição para logs
    int operation_timeout_ms;        // Timeout em milissegundos
    bool operation_completed;        // Flag de operação completada

    // TODO: Métodos auxiliares para implementação futura com sensores
    //
    // /**
    //  * Obtém o thrust total dos motores via ROS2.
    //  */
    // float getTotalMotorThrust() const {
    //     // Implementar leitura dos dados de thrust via tópicos ROS2
    //     // Exemplo: subscriber para /px4/vehicle_status ou similar
    //     return 0.0f;
    // }
    //
    // /**
    //  * Calcula thrust baseline para altitude atual.
    //  */
    // float getBaselineThrustForCurrentAltitude() const {
    //     // Implementar cálculo baseado na altitude e peso do drone
    //     // Thrust necessário = (peso_drone * g) / cos(ângulos_inclinação)
    //     return 0.0f;
    // }
    //
    // static constexpr float PACKAGE_WEIGHT_THRUST_THRESHOLD = 0.5f; // N ou % do thrust total
};
