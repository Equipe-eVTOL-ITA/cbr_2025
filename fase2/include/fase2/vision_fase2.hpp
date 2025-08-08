#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include "Base.hpp"

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
    VisionNode() : Node("fase2_vision") {
        // QoS optimizado para visão computacional
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        this->declare_parameter<double>("timeout", 10.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());
        
        detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/vertical_camera/classification",
            vision_qos,
            [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                detections_.clear();
                this->detection_last_update_ = std::chrono::steady_clock::now();

                if(msg->detections.empty()){
                    this->is_there_detection_ = false;
                    return;
                }

                this->is_there_detection_ = true;
                this->valid_detection_last_update_ = std::chrono::steady_clock::now();

                for (const auto& detection : msg->detections) {
                    BoundingBox bbox;
                    bbox.center_x = detection.bbox.center.position.x;
                    bbox.center_y = detection.bbox.center.position.y;
                    bbox.width = detection.bbox.size_x;
                    bbox.height = detection.bbox.size_y;
                    bbox.confidence = detection.results[0].hypothesis.score;
                    bbox.class_id = detection.results[0].hypothesis.class_id;
                    bbox.timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
                    
                    detections_.push_back(bbox);
                }

                this->computeBboxes();
            }
        );

        base_detection_pub_ = this->create_publisher<custom_msgs::msg::BaseDetection>("/telemetry/bases", 10);
        
        std::string timeout_str = std::to_string(timeout_.count());
        RCLCPP_INFO(this->get_logger(), "Vision node initialized successfully, timeout: %s seconds", timeout_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Vision node initialized successfully");

    }

    double lastDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - detection_last_update_).count();
    }

    double lastBaseDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - valid_detection_last_update_).count();
    }

    BoundingBox getClosestBbox(){
        return this->closest_bbox_;
    }
    
    float getMinDistance(){
        return this->min_distance_;
    }

    bool isThereDetection(){
        if (this->lastDetectionTime() > this->timeout_.count())
            return false;
        return this->is_there_detection_;
    }

    std::vector<BoundingBox> getDetections() {
        return this->detections_;
    }

    void publishBaseDetection(const std::string& base_type,
                            const Eigen::Vector2d& position,
                            float mean_base_height = -0.1,
                            float confidence = 1.0f,
                            uint32_t detection_id = 0) 
    {
        auto msg = custom_msgs::msg::BaseDetection();
        msg.header.stamp = this->get_clock()->now();
        msg.position.x = position.x();
        msg.position.y = position.y(); 
        msg.position.z = mean_base_height;
        msg.base_type = base_type;
        msg.confidence = confidence;
        msg.detection_id = detection_id;
        
        base_detection_pub_->publish(msg);
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    std::vector<BoundingBox> detections_;    
    std::chrono::steady_clock::time_point detection_last_update_;
    std::chrono::steady_clock::time_point valid_detection_last_update_;
    std::chrono::duration<double> timeout_{10.0};

    bool is_there_detection_{false};
    BoundingBox closest_bbox_;
    float min_distance_{0.0f};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<custom_msgs::msg::BaseDetection>::SharedPtr base_detection_pub_;


    void computeBboxes(){
        Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});

        if (this->detections_.empty()) {
            this->is_there_detection_ = false;
            return;
        }

        this->is_there_detection_ = true;
        float min_distance = 2.0f;
        BoundingBox closest_bbox;

        for (const auto& bbox : this->detections_) {
            double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
            if (distance < min_distance) {
                min_distance = distance;
                closest_bbox = bbox;
            }                    
        }
        this->closest_bbox_ = closest_bbox;
        this->min_distance_ = min_distance;
    }    
};
