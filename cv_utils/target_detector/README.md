# Target Detection Framework

Uma arquitetura generalizada e extensível para detecção de objetos em aplicações de visão computacional para drones.

## Arquitetura

### TargetDetector (Classe Abstrata Base)
- **Funcionalidades comuns**: Configuração ROS2, processamento de imagens, publicação de detecções
- **Métodos abstratos**: `_setup_color_parameters()` e `detect()` devem ser implementados pelas classes filhas
- **Sistema de debug**: Integração com a classe `ImageDebug` para visualização

### Classes Implementadas

#### BaseDetector
- **Objetivo**: Detectar bases de pouso (landing pads) com cores azul e amarelo
- **Estratégia**: Análise de regiões que combinam ambas as cores
- **Saída**: Bounding boxes rotacionados com ângulo de orientação

#### PackageDetector  
- **Objetivo**: Detectar pacotes cinza sobre as bases
- **Estratégia**: Segmentação de cor única com análise de forma e orientação
- **Saída**: Bounding boxes com score de confiança baseado em métricas de forma

### ImageDebug
- **Full Debug Mode**: Publica máscaras de cor e bounding boxes separadamente
- **Light Debug Mode**: Publica imagem combinada comprimida para telemetria
- **Rate Control**: Controla taxa de publicação para economizar banda

## Como Usar

### 1. Executar Detector de Bases
```python
from target_detector import BaseDetector, run_detector

detector = BaseDetector()
run_detector(detector)
```

### 2. Executar Detector de Pacotes
```python
from target_detector import PackageDetector, run_detector

detector = PackageDetector()
run_detector(detector)
```

### 3. Criar Novo Detector
```python
from target_detector import TargetDetector
import numpy as np

class MyDetector(TargetDetector):
    def __init__(self):
        super().__init__('my_detector')
    
    def _setup_color_parameters(self):
        # Configure suas cores HSV aqui
        color_lower = {'h': 0, 's': 100, 'v': 100}
        color_upper = {'h': 10, 's': 255, 'v': 255}
        
        self.my_lower, self.my_upper, self.kernel_size, self.iterations = \
            self.setup_color_parameters('my_color', color_lower, color_upper)
    
    def detect(self, image: np.ndarray) -> Tuple[List[Dict], Dict]:
        # Implementar sua lógica de detecção aqui
        detections = []
        debug_images = {'mask_debug': None, 'bbox_debug': image.copy()}
        
        return detections, debug_images
```

## Parâmetros ROS2

### Parâmetros Comuns (TargetDetector)
```yaml
image_topic: '/vertical_camera/image_raw'  # Tópico de entrada das imagens
detection_topic: '/detections'             # Tópico de saída das detecções

# Debug configuration
full_debug_mode: false                     # Ativa debug completo
light_debug_mode: true                     # Ativa debug leve
light_debug_size: 400                      # Tamanho da imagem de debug
light_debug_quality: 80                    # Qualidade JPEG (0-100)
```

### Parâmetros do BaseDetector
```yaml
# Cor azul (HSV)
blue_lower_h: 100
blue_lower_s: 80
blue_lower_v: 50
blue_upper_h: 130
blue_upper_s: 255
blue_upper_v: 255
blue_morph_kernel_size: 5
blue_morph_iterations: 3

# Cor amarela (HSV)
yellow_lower_h: 20
yellow_lower_s: 100
yellow_lower_v: 100
yellow_upper_h: 30
yellow_upper_s: 255
yellow_upper_v: 255
yellow_morph_kernel_size: 3
yellow_morph_iterations: 2

# Detecção combinada
combined_min_area: 0.001                   # Área mínima normalizada
combined_max_area: 1.0                     # Área máxima normalizada
combined_aspect_ratio_min: 0.5             # Aspect ratio mínimo
combined_aspect_ratio_max: 2.0             # Aspect ratio máximo
combination_kernel_size: 15                # Kernel para combinação
combination_iterations: 2                  # Iterações de dilatação
```

### Parâmetros do PackageDetector
```yaml
# Cor cinza (HSV)
gray_lower_h: 0
gray_lower_s: 0
gray_lower_v: 50
gray_upper_h: 180
gray_upper_s: 50
gray_upper_v: 150
gray_morph_kernel_size: 5
gray_morph_iterations: 2

# Detecção de pacotes
package_min_area: 0.0005                   # Área mínima normalizada
package_max_area: 0.5                      # Área máxima normalizada
package_aspect_ratio_min: 0.3              # Aspect ratio mínimo
package_aspect_ratio_max: 3.0              # Aspect ratio máximo
package_min_contour_points: 50             # Pontos mínimos no contorno
```

## Tópicos ROS2

### Entradas
- `/vertical_camera/image_raw` (sensor_msgs/Image): Imagens da câmera

### Saídas
- `/detections` (vision_msgs/Detection2DArray): Detecções encontradas
- `/debug/mask` (sensor_msgs/Image): Debug das máscaras de cor (se habilitado)
- `/debug/bbox` (sensor_msgs/Image): Debug dos bounding boxes (se habilitado)
- `/telemetry/camera_debug/compressed` (sensor_msgs/CompressedImage): Debug combinado para telemetria

## Formato das Detecções

Cada detecção contém:

```python
{
    'bbox': (x, y, w, h),              # Bounding box alinhado aos eixos
    'center': (cx, cy),                # Centro do bounding box rotacionado
    'size': (width, height),           # Dimensões do bounding box rotacionado
    'angle': angle_rad,                # Ângulo em radianos
    'area': area_pixels,               # Área em pixels
    'aspect_ratio': ratio,             # Razão largura/altura
    'class_id': 'landing_pad',         # ID da classe
    'confidence': confidence           # Score de confiança [0, 1]
}
```

## Vantagens da Arquitetura

1. **Reutilização de código**: Funcionalidades comuns centralizadas
2. **Extensibilidade**: Fácil criação de novos detectores
3. **Polimorfismo**: Tratamento uniforme de diferentes detectores
4. **Debug centralizado**: Sistema de visualização consistente
5. **Configurabilidade**: Parâmetros ROS2 para ajuste fino
6. **Performance**: Debug otimizado para aplicações robóticas

## Compatibilidade

Esta implementação mantém compatibilidade total com:
- **VisionNode C++**: Mesmo formato de coordenadas normalizadas
- **Sistema de telemetria**: Imagens debug comprimidas
- **Pipeline existente**: Mesmos tópicos e mensagens ROS2
