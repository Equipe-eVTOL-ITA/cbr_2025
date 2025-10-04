# VisionNode Integration with TargetDetector Framework

Este documento explica como usar o `VisionNode` (C++) integrado com o framework `TargetDetector` (Python) para detectar tanto bases quanto pacotes simultaneamente.

## Arquitetura do Sistema

```
┌─────────────────┐    /base_detector/detections    ┌──────────────┐
│   BaseDetector  │──────────────────────────────────▶│              │
│    (Python)     │                                   │  VisionNode  │
└─────────────────┘                                   │    (C++)     │
                                                      │              │
┌─────────────────┐  /package_detector/detections    │              │
│ PackageDetector │──────────────────────────────────▶│              │
│    (Python)     │                                   └──────────────┘
└─────────────────┘                                          │
                                                             │
                    ┌─────────────────────────────────────────▼
                    │ FSM/Control Node queries VisionNode for:
                    │ - Base positions (landing pads)
                    │ - Package positions  
                    │ - Detection status and timing
                    └─────────────────────────────────────────
```

## Principais Modificações

### VisionNode (C++)

#### Novos Parâmetros
```cpp
// Configuração de tópicos
"base_detection_topic": "/base_detector/detections"
"package_detection_topic": "/package_detector/detections" 
"enable_base_detection": true
"enable_package_detection": true
```

#### Novos Métodos de Consulta
```cpp
// Status de detecções
bool isThereBaseDetection()           // Há bases detectadas?
bool isTherePackageDetection()        // Há pacotes detectados?
bool isThereDetection()               // Há qualquer detecção? (bases OU pacotes)

// Timing
double lastBaseDetectionTime()        // Tempo desde última detecção de base
double lastPackageDetectionTime()     // Tempo desde última detecção de pacote

// Detecções mais próximas
BoundingBox getClosestBaseBbox()      // Base mais próxima do centro
BoundingBox getClosestPackageBbox()   // Pacote mais próximo do centro
float getMinBaseDistance()            // Distância à base mais próxima
float getMinPackageDistance()         // Distância ao pacote mais próximo

// Todas as detecções
std::vector<BoundingBox> getBaseDetections()     // Todas as bases
std::vector<BoundingBox> getPackageDetections() // Todos os pacotes
std::vector<BoundingBox> getAllDetections()     // Todas as detecções
```

#### Novos Métodos de Posicionamento 3D
```cpp
// Posições 3D de bases
Eigen::Vector3d getClosestBasePosition(drone_pos, drone_rpy, height, accurate)

// Posições 3D de pacotes
Eigen::Vector3d getClosestPackagePosition(drone_pos, drone_rpy, height, accurate)
Eigen::Vector3d getApproximatePackage(drone_pos, drone_rpy, bbox, height)
Eigen::Vector3d getAccuratePackage(drone_pos, drone_rpy, bbox)

// Publicação automática
void publishClosestBaseDetection(drone_pos, drone_rpy, height, accurate)
```

## Como Usar

### 1. Executar os Detectores Python

```bash
# Terminal 1: BaseDetector
ros2 run cv_utils base_detector --ros-args --params-file vision_config.yaml

# Terminal 2: PackageDetector  
ros2 run cv_utils package_detector --ros-args --params-file vision_config.yaml
```

### 2. Executar o VisionNode C++

```bash
# Terminal 3: VisionNode
ros2 run fase2 vision_node --ros-args --params-file vision_config.yaml
```

### 3. Usar no Código da FSM

```cpp
#include "fase2/aux/vision_fase2.hpp"

class MyFSM {
    VisionNode vision_node;
    
    void update() {
        // Verificar se há bases detectadas
        if (vision_node.isThereBaseDetection()) {
            auto base_pos = vision_node.getClosestBasePosition(
                drone_position, drone_orientation, 0.0f, true);
            RCLCPP_INFO(logger, "Base at: [%.2f, %.2f, %.2f]",
                       base_pos.x(), base_pos.y(), base_pos.z());
        }
        
        // Verificar se há pacotes detectados
        if (vision_node.isTherePackageDetection()) {
            auto package_pos = vision_node.getClosestPackagePosition(
                drone_position, drone_orientation, 0.1f, true);
            RCLCPP_INFO(logger, "Package at: [%.2f, %.2f, %.2f]",
                       package_pos.x(), package_pos.y(), package_pos.z());
        }
        
        // Publicar detecção de base automaticamente
        vision_node.publishClosestBaseDetection(
            drone_position, drone_orientation, 0.0f, true);
    }
};
```

## Diferenciação por Tipo de Objeto

### Por `class_id`
- **Bases**: `class_id = "landing_pad"`
- **Pacotes**: `class_id = "package"`

### Por Tópico de Subscribe
- **Bases**: `/base_detector/detections`
- **Pacotes**: `/package_detector/detections`

### Por Métodos Específicos
```cpp
// Específico para bases
if (vision_node.isThereBaseDetection()) {
    auto base = vision_node.getClosestBaseBbox();
    // base.class_id == "landing_pad"
}

// Específico para pacotes  
if (vision_node.isTherePackageDetection()) {
    auto package = vision_node.getClosestPackageBbox();
    // package.class_id == "package"
}
```

## Compatibilidade com Código Existente

### Métodos Legacy (Mantidos)
```cpp
// Estes métodos ainda funcionam e retornam bases por compatibilidade
BoundingBox getClosestBbox()          // = getClosestBaseBbox()
float getMinDistance()                // = getMinBaseDistance()
std::vector<BoundingBox> getDetections() // = getBaseDetections()
```

### Transição Gradual
1. **Código existente** continua funcionando sem mudanças
2. **Novo código** pode usar métodos específicos para bases e pacotes
3. **Migration path** clara para atualizar código existente

## Configuração YAML

```yaml
vision_node:
  ros__parameters:
    # Habilitar ambos os tipos de detecção
    enable_base_detection: true
    enable_package_detection: true
    
    # Tópicos de entrada
    base_detection_topic: "/base_detector/detections"
    package_detection_topic: "/package_detector/detections"
    
    # Timeout para detecções
    timeout: 10.0
```

## Fluxo de Dados

1. **BaseDetector** analisa imagem → publica `/base_detector/detections`
2. **PackageDetector** analisa imagem → publica `/package_detector/detections`
3. **VisionNode** subscreve ambos os tópicos → armazena separadamente
4. **FSM** consulta VisionNode → obtém informações integradas

## Vantagens

- ✅ **Detecção simultânea** de bases e pacotes
- ✅ **Diferenciação automática** por tipo de objeto
- ✅ **Compatibilidade total** com código existente
- ✅ **Configuração flexível** (pode desabilitar um tipo)
- ✅ **Performance otimizada** (processamento paralelo)
- ✅ **Debug separado** para cada tipo de objeto

## Estados da FSM com Detecção Integrada

```cpp
enum class MissionState {
    SEARCH_BASE,     // Procura bases usando isThereBaseDetection()
    APPROACH_BASE,   // Aproxima da base usando getClosestBasePosition()
    SEARCH_PACKAGE,  // Procura pacotes usando isTherePackageDetection()  
    APPROACH_PACKAGE,// Aproxima do pacote usando getClosestPackagePosition()
    PICKUP_PACKAGE,  // Pega o pacote
    RETURN_BASE,     // Retorna à base
    DELIVER_PACKAGE  // Entrega o pacote
};
```

Esta integração permite uma missão completa de "buscar base → pegar pacote → entregar na base" com detecção visual robusta e diferenciada para cada tipo de objeto!
