#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/states/align_package_state.hpp"
#include "fase2/states/align_base_state.hpp"
#include "fase2/states/package_state.hpp"

/**
 * Exemplo de integração do PackageState com estados de alinhamento
 * em uma missão completa de coleta e entrega.
 */

class PackageMissionFSM {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    fsm::Blackboard blackboard;
    
    // Estados da missão
    AlignPackageState align_package_state;
    AlignBaseState align_base_state;
    PackageState package_pickup_state;
    PackageState package_drop_state;

public:
    PackageMissionFSM(std::shared_ptr<Drone> drone_ptr, std::shared_ptr<VisionNode> vision_ptr) 
        : drone(drone_ptr), vision(vision_ptr) {
        setupBlackboard();
    }

private:
    void setupBlackboard() {
        // Objetos principais
        blackboard.set("drone", drone);
        blackboard.set("vision", vision);
        
        // Parâmetros de alinhamento (já configurados anteriormente)
        blackboard.set("align_tolerance", 0.05f);           // Tolerância mais restrita para pickup
        blackboard.set("max_horizontal_velocity", 0.3f);    // Velocidade mais baixa para precisão
        blackboard.set("detection_timeout", 10.0f);
        
        // Configurações específicas do PackageState
        blackboard.set("package_operation_timeout", 6.0f);  // 6 segundos para operação da garra
        
        // Estados da missão
        blackboard.set("package_state", std::string("get_package")); // Inicialmente buscando package
    }

public:
    /**
     * Demonstra sequência completa de coleta de package.
     */
    void demonstratePackagePickup() {
        drone->log("=== INICIANDO COLETA DE PACKAGE ===");
        
        // FASE 1: Alinhar com package
        drone->log("FASE 1: Alinhando com package...");
        
        // Configurar operação de alinhamento para package
        blackboard.set("package_state", std::string("get_package"));
        
        align_package_state.on_enter(blackboard);
        
        // Simular processo de alinhamento
        std::string alignment_result = "";
        while (alignment_result != "PRECISELY_ALIGNED") {
            alignment_result = align_package_state.act(blackboard);
            
            if (alignment_result == "LOST PACKAGE") {
                drone->log("Package perdido durante alinhamento!");
                return;
            }
            
            // Simular tempo de processamento
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        align_package_state.on_exit(blackboard);
        drone->log("Alinhamento com package concluído!");
        
        // FASE 2: Pegar package
        drone->log("FASE 2: Coletando package...");
        
        // Configurar operação de pickup
        blackboard.set("package_operation", std::string("pickup"));
        
        package_pickup_state.on_enter(blackboard);
        
        // Executar operação de pickup
        std::string pickup_result = "";
        while (pickup_result != "PACKAGE_PICKED_UP") {
            pickup_result = package_pickup_state.act(blackboard);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Verificar se package foi pego com sucesso
        bool package_secured = package_pickup_state.isPackageManipulated();
        drone->log("Package pickup status: " + std::string(package_secured ? "SUCCESS" : "FAILED"));
        
        package_pickup_state.on_exit(blackboard);
        
        if (package_secured) {
            drone->log("=== PACKAGE COLETADO COM SUCESSO ===");
        } else {
            drone->log("=== FALHA NA COLETA DO PACKAGE ===");
        }
    }
    
    /**
     * Demonstra sequência completa de entrega de package.
     */
    void demonstratePackageDelivery() {
        drone->log("=== INICIANDO ENTREGA DE PACKAGE ===");
        
        // FASE 1: Alinhar com base de entrega
        drone->log("FASE 1: Alinhando com base de entrega...");
        
        // Configurar operação de alinhamento para base
        blackboard.set("package_state", std::string("deliver_package"));
        
        align_base_state.on_enter(blackboard);
        
        // Simular processo de alinhamento
        std::string alignment_result = "";
        while (alignment_result != "PRECISELY_ALIGNED") {
            alignment_result = align_base_state.act(blackboard);
            
            if (alignment_result == "LOST BASE") {
                drone->log("Base perdida durante alinhamento!");
                return;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        align_base_state.on_exit(blackboard);
        drone->log("Alinhamento com base concluído!");
        
        // FASE 2: Soltar package
        drone->log("FASE 2: Entregando package...");
        
        // Configurar operação de drop
        blackboard.set("package_operation", std::string("drop"));
        
        package_drop_state.on_enter(blackboard);
        
        // Executar operação de drop
        std::string drop_result = "";
        while (drop_result != "PACKAGE_DROPPED") {
            drop_result = package_drop_state.act(blackboard);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Verificar se package foi solto com sucesso
        bool package_released = package_drop_state.isPackageManipulated();
        drone->log("Package drop status: " + std::string(package_released ? "SUCCESS" : "FAILED"));
        
        package_drop_state.on_exit(blackboard);
        
        if (package_released) {
            drone->log("=== PACKAGE ENTREGUE COM SUCESSO ===");
        } else {
            drone->log("=== FALHA NA ENTREGA DO PACKAGE ===");
        }
    }
    
    /**
     * Demonstra monitoramento durante operações.
     */
    void monitorPackageOperations() {
        drone->log("=== MONITORAMENTO DE OPERAÇÕES ===");
        
        // Exemplo de monitoramento durante pickup
        blackboard.set("package_operation", std::string("pickup"));
        package_pickup_state.on_enter(blackboard);
        
        // Monitorar progresso da operação
        for (int i = 0; i < 50; i++) {  // ~5 segundos de monitoramento
            std::string result = package_pickup_state.act(blackboard);
            
            // Log de status periódico
            if (i % 10 == 0) {
                drone->log("Operation: " + package_pickup_state.getOperation());
                drone->log("Elapsed time: " + std::to_string(package_pickup_state.getElapsedTime()) + "ms");
                drone->log("Completed: " + std::string(package_pickup_state.isOperationCompleted() ? "YES" : "NO"));
                drone->log("Package manipulated: " + std::string(package_pickup_state.isPackageManipulated() ? "YES" : "NO"));
            }
            
            if (result == "PACKAGE_PICKED_UP") {
                drone->log("Pickup operation completed!");
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        package_pickup_state.on_exit(blackboard);
    }
    
    /**
     * Demonstra configuração dinâmica de parâmetros.
     */
    void demonstrateParameterConfiguration() {
        drone->log("=== CONFIGURAÇÃO DINÂMICA ===");
        
        // Configurar timeout mais longo para operações críticas
        blackboard.set("package_operation_timeout", 10.0f);
        drone->log("Timeout configurado para 10 segundos");
        
        // Configurar tolerâncias mais restritivas para pickup
        blackboard.set("align_tolerance", 0.03f);
        drone->log("Tolerância de alinhamento: 3cm");
        
        // Configurar velocidades mais baixas para precisão
        blackboard.set("max_horizontal_velocity", 0.2f);
        drone->log("Velocidade máxima: 0.2 m/s");
        
        // Teste com configuração personalizada
        blackboard.set("package_operation", std::string("pickup"));
        
        package_pickup_state.on_enter(blackboard);
        drone->log("Estado configurado com parâmetros personalizados");
        package_pickup_state.on_exit(blackboard);
    }
    
    /**
     * Executa missão completa de coleta e entrega.
     */
    void executeFullMission() {
        drone->log("========================================");
        drone->log("=== MISSÃO COMPLETA DE PACKAGE ===");
        drone->log("========================================");
        
        try {
            // Coleta
            demonstratePackagePickup();
            
            // Pausa entre operações
            std::this_thread::sleep_for(std::chrono::seconds(2));
            
            // Entrega
            demonstratePackageDelivery();
            
            drone->log("========================================");
            drone->log("=== MISSÃO CONCLUÍDA ===");
            drone->log("========================================");
            
        } catch (const std::exception& e) {
            drone->log("ERRO NA MISSÃO: " + std::string(e.what()));
        }
    }
};

/**
 * Função principal de demonstração.
 */
int main() {
    // Criar objetos principais
    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    
    // Criar FSM de missão com PackageState
    PackageMissionFSM mission(drone, vision);
    
    // Demonstrar diferentes aspectos do sistema
    mission.demonstrateParameterConfiguration();
    mission.monitorPackageOperations();
    mission.executeFullMission();
    
    return 0;
}
