#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/states/align_base_state.hpp"
#include "fase2/states/align_package_state.hpp"

/**
 * Exemplo de integração dos novos estados de alinhamento na FSM da Fase 2.
 * 
 * Este exemplo mostra como usar AlignBaseState e AlignPackageState
 * em uma missão completa de coleta e entrega de packages.
 */

class MissionFSM {
private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    fsm::Blackboard blackboard;
    
    // Estados de alinhamento especializados
    AlignBaseState align_base_state;
    AlignPackageState align_package_state;

public:
    MissionFSM(std::shared_ptr<Drone> drone_ptr, std::shared_ptr<VisionNode> vision_ptr) 
        : drone(drone_ptr), vision(vision_ptr) {
        
        // Configurar blackboard com parâmetros dos estados
        setupBlackboard();
        
        // Configurar FSM com os novos estados
        setupStateMachine();
    }

private:
    void setupBlackboard() {
        // Objetos principais
        blackboard.set("drone", drone);
        blackboard.set("vision", vision);
        
        // === Parâmetros de Alinhamento ===
        blackboard.set("align_tolerance", 0.1f);
        blackboard.set("max_horizontal_velocity", 0.5f);
        blackboard.set("align_descent_velocity", 0.2f);
        blackboard.set("detection_timeout", 5.0f);
        
        // === Controladores PID ===
        // PID para posição (X/Y)
        blackboard.set("pid_pos_kp", 1.0f);
        blackboard.set("pid_pos_ki", 0.1f);
        blackboard.set("pid_pos_kd", 0.05f);
        blackboard.set("setpoint", 0.0f);
        
        // PID para yaw (apenas packages)
        blackboard.set("pid_yaw_kp", 2.0f);
        blackboard.set("pid_yaw_ki", 0.0f);
        blackboard.set("pid_yaw_kd", 0.1f);
        
        // === Offsets Específicos ===
        // Offset da câmera para alinhamento com bases
        blackboard.set("base_align_offset_x", 0.05f);
        blackboard.set("base_align_offset_y", 0.02f);
        
        // Offset da garra para alinhamento com packages
        blackboard.set("package_align_offset_x", 0.15f);
        blackboard.set("package_align_offset_y", 0.0f);
        
        // === Parâmetros Específicos ===
        blackboard.set("height_to_ground", 1.0f);
        blackboard.set("mean_base_height", 0.0f);
        blackboard.set("mean_package_height", 0.1f);
        blackboard.set("yaw_align_tolerance", 0.087f); // ~5 graus
        blackboard.set("max_yaw_rate", 0.5f);
        
        // Estado da missão
        blackboard.set("package_state", std::string("get_package"));
    }
    
    void setupStateMachine() {
        // === FASE 1: BUSCA E COLETA DO PACKAGE ===
        
        // Transição: TAKEOFF -> SEARCH_PACKAGE
        // Estado: Buscar package na área designada
        
        // Transição: SEARCH_PACKAGE -> ALIGN_PACKAGE
        // Usar AlignPackageState para alinhamento preciso com package
        // Inclui alinhamento rotacional para orientação correta
        
        // Transição: ALIGN_PACKAGE -> PICKUP_PACKAGE
        // Estado: Descer e ativar garra para pegar package
        
        // === FASE 2: BUSCA DA BASE DE ENTREGA ===
        
        // Atualizar estado da missão para entrega
        // blackboard.set("package_state", std::string("deliver_package"));
        
        // Transição: PICKUP_PACKAGE -> SEARCH_DELIVERY_BASE
        // Estado: Buscar base de entrega
        
        // Transição: SEARCH_DELIVERY_BASE -> ALIGN_BASE
        // Usar AlignBaseState para alinhamento com base de entrega
        // Considera offset da câmera, sem necessidade de alinhamento rotacional
        
        // Transição: ALIGN_BASE -> DELIVER_PACKAGE
        // Estado: Descer e soltar package na base
        
        // Transição: DELIVER_PACKAGE -> LAND
        // Estado: Aterrissar na base de entrega
    }

public:
    /**
     * Exemplo de execução da missão completa.
     */
    void executeMission() {
        drone->log("=== INICIANDO MISSÃO DE COLETA E ENTREGA ===");
        
        // FASE 1: Coleta do Package
        drone->log("FASE 1: Buscando package...");
        
        // Quando package for detectado, FSM transicionará para AlignPackageState
        // AlignPackageState irá:
        // 1. Usar vision->isTherePackageDetection() para verificar detecção
        // 2. Obter posição com vision->getClosestPackagePosition()
        // 3. Aplicar offset da garra (package_align_offset_x/y)
        // 4. Alinhar posição E rotação usando PIDs
        // 5. Retornar "PRECISELY ALIGNED" quando alinhado
        
        drone->log("Package detectado! Iniciando alinhamento...");
        // AlignPackageState ativo aqui
        
        drone->log("Alinhamento com package concluído! Coletando...");
        
        // FASE 2: Entrega na Base
        blackboard.set("package_state", std::string("deliver_package"));
        drone->log("FASE 2: Buscando base de entrega...");
        
        // Quando base for detectada, FSM transicionará para AlignBaseState
        // AlignBaseState irá:
        // 1. Usar vision->isThereBaseDetection() para verificar detecção
        // 2. Obter posição com vision->getClosestBasePosition()
        // 3. Aplicar offset da câmera (base_align_offset_x/y)
        // 4. Alinhar apenas posição (sem rotação)
        // 5. Publicar telemetria da base detectada
        // 6. Retornar "PRECISELY ALIGNED" quando alinhado
        
        drone->log("Base de entrega detectada! Iniciando alinhamento...");
        // AlignBaseState ativo aqui
        
        drone->log("Alinhamento com base concluído! Entregando package...");
        
        drone->log("=== MISSÃO CONCLUÍDA COM SUCESSO ===");
    }
    
    /**
     * Exemplo de debug - verificar estado atual dos detectores.
     */
    void debugDetectionStatus() {
        drone->log("=== STATUS DOS DETECTORES ===");
        
        // Status de detecção de bases
        bool base_detected = vision->isThereBaseDetection();
        double base_last_time = vision->lastBaseDetectionTime();
        drone->log("Base detection: " + std::string(base_detected ? "ATIVA" : "INATIVA") + 
                  " (última: " + std::to_string(base_last_time) + "s atrás)");
        
        if (base_detected) {
            auto base_bbox = vision->getClosestBaseBbox();
            drone->log("Closest base: center=[" + std::to_string(base_bbox.center_x) + 
                      "," + std::to_string(base_bbox.center_y) + 
                      "] confidence=" + std::to_string(base_bbox.confidence));
        }
        
        // Status de detecção de packages
        bool package_detected = vision->isTherePackageDetection();
        double package_last_time = vision->lastPackageDetectionTime();
        drone->log("Package detection: " + std::string(package_detected ? "ATIVA" : "INATIVA") + 
                  " (última: " + std::to_string(package_last_time) + "s atrás)");
        
        if (package_detected) {
            auto package_bbox = vision->getClosestPackageBbox();
            drone->log("Closest package: center=[" + std::to_string(package_bbox.center_x) + 
                      "," + std::to_string(package_bbox.center_y) + 
                      "] rotation=" + std::to_string(package_bbox.rotation * 180.0 / M_PI) + "°");
        }
    }
    
    /**
     * Exemplo de configuração dinâmica de offsets.
     */
    void configureOffsets(float camera_offset_x, float camera_offset_y,
                         float grappler_offset_x, float grappler_offset_y) {
        drone->log("Configurando offsets dinâmicos...");
        
        // Offsets da câmera (para bases)
        blackboard.set("base_align_offset_x", camera_offset_x);
        blackboard.set("base_align_offset_y", camera_offset_y);
        
        // Offsets da garra (para packages)
        blackboard.set("package_align_offset_x", grappler_offset_x);
        blackboard.set("package_align_offset_y", grappler_offset_y);
        
        drone->log("Offsets configurados - Câmera: [" + std::to_string(camera_offset_x) + 
                  "," + std::to_string(camera_offset_y) + 
                  "] Garra: [" + std::to_string(grappler_offset_x) + 
                  "," + std::to_string(grappler_offset_y) + "]");
    }
};

/**
 * Função principal de exemplo.
 */
int main() {
    // Criar objetos principais
    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    
    // Criar FSM com novos estados de alinhamento
    MissionFSM mission_fsm(drone, vision);
    
    // Configurar offsets específicos do drone
    mission_fsm.configureOffsets(
        0.05f, 0.02f,  // Offset da câmera
        0.15f, 0.0f    // Offset da garra
    );
    
    // Executar missão completa
    mission_fsm.executeMission();
    
    return 0;
}
