#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

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

        package_sub_ = this->create_subscription<vision_msgs::msg::Detection2D>(
            "/package/classification",
            vision_qos,
            [this](const vision_msgs::msg::Detection2D::SharedPtr msg) {
                this->package_last_update_ = std::chrono::steady_clock::now();

                BoundingBox bbox;
                bbox.center_x = msg->bbox.center.position.x;
                bbox.center_y = msg->bbox.center.position.y;
                bbox.width = msg->bbox.size_x;
                bbox.height = msg->bbox.size_y;
                bbox.confidence = msg->results[0].hypothesis.score;
                bbox.class_id = msg->results[0].hypothesis.class_id;
                bbox.timestamp = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;

                this->package_detection_ = bbox;
            }
        );
        
        // Publisher for base detection telemetry
        base_detection_pub_ = this->create_publisher<custom_msgs::msg::BaseDetection>("/telemetry/bases", 10);
        
        RCLCPP_INFO(this->get_logger(), "Vision node initialized successfully");
    }

    // BASE GETTERS ------------------------------------------------------------------------

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

    std::vector<BoundingBox> getDetections(){
        return this->detections_;
    }

    // -------------------------------------------------------------------------------------


    // PACKAGE GETTERS ---------------------------------------------------------------------

    double lastPackageTime(){
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - package_last_update_).count();
    }

    BoundingBox getPackageBbox(){
        return this->package_detection_;
    }

    bool isTherePackage(){
        return this->lastPackageTime() < this->timeout_.count() ? true : false;
    }

    // -------------------------------------------------------------------------------------
    
    // Method to publish base detection telemetry
    void publishBaseDetection(const Eigen::Vector2d& position, const std::string& base_type, 
                            float confidence = 1.0f, uint32_t detection_id = 0) {
        auto msg = custom_msgs::msg::BaseDetection();
        msg.header.stamp = this->get_clock()->now();
        msg.position.x = position.x();
        msg.position.y = position.y(); 
        msg.position.z = 0.0; // Ground level
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

    rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr package_sub_;
    BoundingBox package_detection_;
    std::chrono::steady_clock::time_point package_last_update_;
    
    rclcpp::Publisher<custom_msgs::msg::BaseDetection>::SharedPtr base_detection_pub_;

    bool is_there_detection_{false};
    BoundingBox closest_bbox_;
    float min_distance_{0.0f};

    std::chrono::duration<double> timeout_{1.0};

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
