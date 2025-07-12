#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

// Bounding Box structure moved from Drone namespace
struct BoundingBox {
    float center_x;
    float center_y;
    float width;
    float height;
    float confidence;
    std::string class_id;
    int64_t timestamp;
};

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("fase1_vision") {
        // QoS optimizado para visão computacional
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Subscribers para detecções
        vertical_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/vertical_camera/classification", vision_qos,
            std::bind(&VisionNode::verticalDetectionsCallback, this, std::placeholders::_1));
        
        angled_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/angled_camera/classification", vision_qos,
            std::bind(&VisionNode::angledDetectionsCallback, this, std::placeholders::_1));
        
        horizontal_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/horizontal_camera/classification", vision_qos,
            std::bind(&VisionNode::horizontalDetectionsCallback, this, std::placeholders::_1));
        
        // Subscribers para imagens (opcional para debugging)
        vertical_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vertical_camera", vision_qos,
            std::bind(&VisionNode::verticalImageCallback, this, std::placeholders::_1));
        
        angled_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/angled_camera", vision_qos,
            std::bind(&VisionNode::angledImageCallback, this, std::placeholders::_1));
        
        horizontal_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/horizontal_camera", vision_qos,
            std::bind(&VisionNode::horizontalImageCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Vision node initialized successfully");
    }
    
    // Estrutura para resultado de detecção processado
    struct DetectionResult {
        bool has_detection = false;
        BoundingBox closest_bbox;
        float min_distance = 0.0;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    // Getters para detecções brutas
    std::vector<BoundingBox> getVerticalDetections() const {
        return vertical_detections_;
    }
    
    std::vector<BoundingBox> getAngledDetections() const {
        return angled_detections_;
    }
    
    std::vector<BoundingBox> getHorizontalDetections() const {
        return horizontal_detections_;
    }
    
    // Getters para imagens
    cv_bridge::CvImagePtr getVerticalImage() const {
        return vertical_image_;
    }
    
    cv_bridge::CvImagePtr getAngledImage() const {
        return angled_image_;
    }
    
    cv_bridge::CvImagePtr getHorizontalImage() const {
        return horizontal_image_;
    }
    
    // Funcionalidade integrada da classe Detection
    DetectionResult getClosestDetection(const std::string& camera_type, 
                                      const std::string& target_color = "") {
        std::vector<BoundingBox> detections;
        
        // Selecionar detecções baseado no tipo de câmera
        if (camera_type == "vertical") {
            detections = vertical_detections_;
        } else if (camera_type == "angled") {
            detections = angled_detections_;
        } else if (camera_type == "horizontal") {
            detections = horizontal_detections_;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown camera type: %s", camera_type.c_str());
            return DetectionResult{};
        }
        
        return computeClosestBbox(detections, target_color);
    }
    
    // Verificar se há detecções recentes
    bool hasRecentDetections(const std::string& camera_type, double timeout_seconds = 1.0) const {
        auto now = std::chrono::steady_clock::now();
        auto timeout_duration = std::chrono::duration<double>(timeout_seconds);
        
        if (camera_type == "vertical") {
            return (now - vertical_last_update_) < timeout_duration;
        } else if (camera_type == "angled") {
            return (now - angled_last_update_) < timeout_duration;
        } else if (camera_type == "horizontal") {
            return (now - horizontal_last_update_) < timeout_duration;
        }
        
        return false;
    }
    
    // Estatísticas
    void printVisionStats() const {
        RCLCPP_INFO(this->get_logger(), "=== Vision Node Statistics ===");
        RCLCPP_INFO(this->get_logger(), "Vertical detections: %zu", vertical_detections_.size());
        RCLCPP_INFO(this->get_logger(), "Angled detections: %zu", angled_detections_.size());
        RCLCPP_INFO(this->get_logger(), "Horizontal detections: %zu", horizontal_detections_.size());
        RCLCPP_INFO(this->get_logger(), "==============================");
    }

private:
    // Subscribers
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr vertical_detections_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr angled_detections_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr horizontal_detections_sub_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vertical_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr angled_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr horizontal_image_sub_;
    
    // Data storage
    std::vector<BoundingBox> vertical_detections_;
    std::vector<BoundingBox> angled_detections_;
    std::vector<BoundingBox> horizontal_detections_;
    
    cv_bridge::CvImagePtr vertical_image_;
    cv_bridge::CvImagePtr angled_image_;
    cv_bridge::CvImagePtr horizontal_image_;
    
    // Timestamps
    std::chrono::steady_clock::time_point vertical_last_update_;
    std::chrono::steady_clock::time_point angled_last_update_;
    std::chrono::steady_clock::time_point horizontal_last_update_;
    
    // Callbacks para detecções
    void verticalDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        vertical_detections_.clear();
        for (const auto& detection : msg->detections) {
            if (!detection.results.empty()) {
                BoundingBox bbox;
                bbox.center_x = detection.bbox.center.position.x;
                bbox.center_y = detection.bbox.center.position.y;
                bbox.width = detection.bbox.size_x;
                bbox.height = detection.bbox.size_y;
                bbox.confidence = detection.results[0].hypothesis.score;
                bbox.class_id = detection.results[0].hypothesis.class_id;
                bbox.timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
                
                vertical_detections_.push_back(bbox);
            }
        }
        vertical_last_update_ = std::chrono::steady_clock::now();
    }
    
    void angledDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        angled_detections_.clear();
        for (const auto& detection : msg->detections) {
            if (!detection.results.empty()) {
                BoundingBox bbox;
                bbox.center_x = detection.bbox.center.position.x;
                bbox.center_y = detection.bbox.center.position.y;
                bbox.width = detection.bbox.size_x;
                bbox.height = detection.bbox.size_y;
                bbox.confidence = detection.results[0].hypothesis.score;
                bbox.class_id = detection.results[0].hypothesis.class_id;
                bbox.timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
                
                angled_detections_.push_back(bbox);
            }
        }
        angled_last_update_ = std::chrono::steady_clock::now();
    }
    
    void horizontalDetectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        horizontal_detections_.clear();
        for (const auto& detection : msg->detections) {
            if (!detection.results.empty()) {
                BoundingBox bbox;
                bbox.center_x = detection.bbox.center.position.x;
                bbox.center_y = detection.bbox.center.position.y;
                bbox.width = detection.bbox.size_x;
                bbox.height = detection.bbox.size_y;
                bbox.confidence = detection.results[0].hypothesis.score;
                bbox.class_id = detection.results[0].hypothesis.class_id;
                bbox.timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
                
                horizontal_detections_.push_back(bbox);
            }
        }
        horizontal_last_update_ = std::chrono::steady_clock::now();
    }
    
    // Callbacks para imagens
    void verticalImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            vertical_image_ = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    void angledImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            angled_image_ = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    void horizontalImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            horizontal_image_ = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }
    
    // Função integrada da classe Detection
    DetectionResult computeClosestBbox(const std::vector<BoundingBox>& bboxes,
                                     const std::string& target_color = "") {
        DetectionResult result;
        result.timestamp = std::chrono::steady_clock::now();
        
        Eigen::Vector2d image_center(0.5, 0.5);
        
        if (bboxes.empty()) {
            result.has_detection = false;
            return result;
        }
        
        float min_distance = 2.0f;
        BoundingBox closest_bbox;
        bool found_target = false;
        
        // Encontrar bbox mais próximo do centro, com filtro de cor opcional
        for (const auto& bbox : bboxes) {
            // Se target_color é especificado, filtrar por cor
            if (!target_color.empty() && bbox.class_id != target_color) {
                continue; // Pular esta detecção se não corresponder à cor alvo
            }
            
            double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_bbox = bbox;
                found_target = true;
            }
        }
        
        // Apenas definir detecção como verdadeira se encontrou a cor alvo (ou sem filtro de cor)
        result.has_detection = found_target;
        if (found_target) {
            result.closest_bbox = closest_bbox;
            result.min_distance = min_distance;
        }
        
        return result;
    }
};
