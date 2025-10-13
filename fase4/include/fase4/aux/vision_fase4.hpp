#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
// #include <std_srvs/srv/trigger.hpp>  // REMOVIDO temporariamente - pacote não instalado
#include <memory>
#include <chrono>
#include <string>

struct Janela {
    float center_x;      // Coordenadas relativas ao centro da imagem (metros no sistema do drone)
    float center_y;      // Coordenadas relativas ao centro da imagem (metros no sistema do drone) 
    float center_z;      // Sempre 0.0 (detecção 2D)
    bool detected;       // Se a janela foi detectada
    int64_t timestamp;   // Timestamp da última detecção
    
    Janela() : center_x(0.0f), center_y(0.0f), center_z(0.0f), detected(false), timestamp(0) {}
    
    Janela(float x, float y, float z = 0.0f, bool det = true, int64_t ts = 0) 
        : center_x(x), center_y(y), center_z(z), detected(det), timestamp(ts) {}
};

struct QRCode {
    float center_x;      // Coordenadas relativas ao centro da imagem (normalizadas)
    float center_y;      // Coordenadas relativas ao centro da imagem (normalizadas)
    float center_z;      // Sempre 0.0 (detecção 2D)
    float width;         // Largura do bounding box (normalizada)
    float height;        // Altura do bounding box (normalizada)
    std::string content; // Conteúdo textual do QR Code
    bool detected;       // Se o QR Code foi detectado
    int64_t timestamp;   // Timestamp da última detecção
    
    QRCode() : center_x(0.0f), center_y(0.0f), center_z(0.0f), width(0.0f), height(0.0f), 
               content(""), detected(false), timestamp(0) {}
    
    QRCode(float x, float y, float w, float h, const std::string& text, float z = 0.0f, 
           bool det = true, int64_t ts = 0) 
        : center_x(x), center_y(y), center_z(z), width(w), height(h), 
          content(text), detected(det), timestamp(ts) {}
};

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("fase4_vision") {
        // QoS for vision
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        this->declare_parameter<double>("timeout", 10.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());
        
        // Subscribers para detecção de janelas (conforme window_detector.py)
        window_centroid_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/centroid",
            vision_qos,
            std::bind(&VisionNode::windowCentroidCallback, this, std::placeholders::_1)
        );
        
        window_detected_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/window_found", 
            vision_qos,
            std::bind(&VisionNode::windowDetectedCallback, this, std::placeholders::_1)
        );

        // Subscribers para detecção de QR Codes (conforme qrcode_detector.py)
        qr_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/vertical_classification",
            vision_qos,
            std::bind(&VisionNode::qrDetectionsCallback, this, std::placeholders::_1)
        );
        
        qr_string_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/qr_code_string",
            vision_qos,
            std::bind(&VisionNode::qrStringCallback, this, std::placeholders::_1)
        );

        // Cliente para serviço de captura de imagem QR (REMOVIDO temporariamente)
        // qr_capture_client_ = this->create_client<std_srvs::srv::Trigger>("/capture_qr_image");

        RCLCPP_INFO(this->get_logger(), "Fase4 Vision node initialized for window and QR Code detection");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /centroid, /window_found, /vertical_classification, and /qr_code_string topics");
    }

    // === Métodos principais para detecção de janelas ===
    
    /**
     * Verifica se há uma janela detectada recentemente
     * @return true se há detecção válida dentro do timeout
     */
    bool isThereJanelaDetection() {
        auto now = std::chrono::steady_clock::now();
        double time_since_detection = std::chrono::duration<double>(now - janela_last_update_).count();
        return (time_since_detection <= timeout_.count()) && current_janela_.detected;
    }
    
    /**
     * Obtém a janela detectada mais recente
     * @return Struct Janela com as coordenadas e status de detecção
     */
    Janela getJanela() const {
        return current_janela_;
    }
    
    /**
     * Obtém o tempo desde a última detecção de janela
     * @return Tempo em segundos desde a última detecção
     */
    double lastJanelaDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - janela_last_update_).count();
    }

    // === Métodos principais para detecção de QR Codes ===
    
    /**
     * Verifica se há um QR Code detectado recentemente
     * @return true se há detecção válida dentro do timeout
     */
    bool isThereQRCodeDetection() {
        auto now = std::chrono::steady_clock::now();
        double time_since_detection = std::chrono::duration<double>(now - qr_last_update_).count();
        return (time_since_detection <= timeout_.count()) && current_qr_.detected;
    }
    
    /**
     * Obtém o QR Code detectado mais recente
     * @return Struct QRCode com as coordenadas, dimensões e conteúdo
     */
    QRCode getQRCode() const {
        return current_qr_;
    }
    
    /**
     * Obtém apenas o conteúdo textual do QR Code
     * @return String com o texto do QR Code ou string vazia se não detectado
     */
    std::string getQRCodeContent() const {
        return current_qr_.content;
    }
    
    /**
     * Obtém o tempo desde a última detecção de QR Code
     * @return Tempo em segundos desde a última detecção
     */
    double lastQRCodeDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - qr_last_update_).count();
    }
    
    /**
     * Captura imagem do QR Code atualmente detectado
     * @return true se captura foi bem-sucedida, false caso contrário
     */
    bool captureQRCodeImage() {
        RCLCPP_WARN(this->get_logger(), "QR Code capture functionality not available - std_srvs not installed");
        return false;  // Retorna false indicando que não funcionou
    }

    // === Callbacks para detecção de janelas ===
    
    /**
     * Callback para receber as coordenadas do centróide da janela
     * Formato: geometry_msgs/Point com coordenadas já convertidas para o sistema do drone
     */
    void windowCentroidCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        current_janela_.center_x = msg->x;
        current_janela_.center_y = msg->y; 
        current_janela_.center_z = msg->z;
        current_janela_.timestamp = this->get_clock()->now().nanoseconds();
        janela_last_update_ = std::chrono::steady_clock::now();
        
        // Log para debug
        RCLCPP_DEBUG(this->get_logger(), 
                    "Window centroid received: x=%.3f, y=%.3f, z=%.3f", 
                    current_janela_.center_x, current_janela_.center_y, current_janela_.center_z);
    }
    
    /**
     * Callback para receber o status de detecção da janela
     * Formato: std_msgs/Bool indicando se há janela detectada
     */
    void windowDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        current_janela_.detected = msg->data;
        janela_last_update_ = std::chrono::steady_clock::now();
        
        if (msg->data) {
            RCLCPP_DEBUG(this->get_logger(), "Window detected!");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No window detected");
            // Reset coordinates when no detection
            current_janela_.center_x = 0.0f;
            current_janela_.center_y = 0.0f;
            current_janela_.center_z = 0.0f;
        }
    }

    // === Callbacks para detecção de QR Codes ===
    
    /**
     * Callback para receber as detecções de QR Codes com bounding boxes
     * Formato: vision_msgs/Detection2DArray com coordenadas normalizadas
     */
    void qrDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        qr_last_update_ = std::chrono::steady_clock::now();
        
        if (!msg->detections.empty()) {
            // Pegar a primeira detecção (assumindo um QR Code por vez)
            const auto& detection = msg->detections[0];
            
            // Verificar se há detecção válida (coordenadas não-zero)
            if (detection.bbox.center.position.x != 0.0 || detection.bbox.center.position.y != 0.0) {
                current_qr_.center_x = detection.bbox.center.position.x;
                current_qr_.center_y = detection.bbox.center.position.y;
                current_qr_.center_z = 0.0f;
                current_qr_.width = detection.bbox.size_x;
                current_qr_.height = detection.bbox.size_y;
                current_qr_.detected = true;
                current_qr_.timestamp = this->get_clock()->now().nanoseconds();
                
                RCLCPP_DEBUG(this->get_logger(), 
                            "QR Code detected at: x=%.3f, y=%.3f, size=%.3fx%.3f", 
                            current_qr_.center_x, current_qr_.center_y,
                            current_qr_.width, current_qr_.height);
            } else {
                // Detecção com coordenadas zero = sem QR Code
                current_qr_.detected = false;
                resetQRCode();
            }
        } else {
            // Nenhuma detecção
            current_qr_.detected = false;
            resetQRCode();
        }
    }
    
    /**
     * Callback para receber o conteúdo textual do QR Code
     * Formato: std_msgs/String com o texto decodificado
     */
    void qrStringCallback(const std_msgs::msg::String::SharedPtr msg) {
        current_qr_.content = msg->data;
        qr_last_update_ = std::chrono::steady_clock::now();
        
        if (!msg->data.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "QR Code content: %s", msg->data.c_str());
        } else {
            RCLCPP_DEBUG(this->get_logger(), "No QR Code content");
        }
    }

private:
    // ==== Subscribers para detecção de janelas ============================================================
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr window_centroid_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr window_detected_sub_;
    
    // ==== Subscribers para detecção de QR Codes =========================================================
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr qr_detections_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_string_sub_;
    
    // ==== Cliente para captura de imagens (REMOVIDO temporariamente) ================================
    // rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr qr_capture_client_;
    
    // Armazenamento das detecções atuais
    Janela current_janela_;
    QRCode current_qr_;
    
    // Controle de timing
    std::chrono::steady_clock::time_point janela_last_update_{};
    std::chrono::steady_clock::time_point qr_last_update_{};
    std::chrono::duration<double> timeout_{10.0};
    
    // Método helper para resetar QR Code
    void resetQRCode() {
        current_qr_.center_x = 0.0f;
        current_qr_.center_y = 0.0f;
        current_qr_.center_z = 0.0f;
        current_qr_.width = 0.0f;
        current_qr_.height = 0.0f;
        current_qr_.content = "";
    }
};
