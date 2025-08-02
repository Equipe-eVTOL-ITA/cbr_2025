#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
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

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/base_markers", 10);
        marker_id_counter_ = 0;
        
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
        return this->is_there_detection_;
    }

    std::vector<BoundingBox> getDetections() {
        return this->detections_;
    }


    void publishKnownBases(const std::vector<Base>& bases, float known_base_radius = 1.0) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        for (size_t i = 0; i < bases.size(); ++i) {
            // Green square for the base
            auto base_marker = createBaseMarker(bases[i].coordinates, 
                                              "known_base_" + std::to_string(i),
                                              0.0, 1.0, 0.0, true);  // Green, persistent
            marker_array.markers.push_back(base_marker);
            
            // Green circle for the radius
            auto circle_marker = createCircleMarker(bases[i].coordinates,
                                                  "known_base_circle_" + std::to_string(i),
                                                  known_base_radius,
                                                  0.0, 1.0, 0.0, true);  // Green, persistent
            marker_array.markers.push_back(circle_marker);
        }
        
        marker_pub_->publish(marker_array);
    }

    void publishFirstEstimateBase(const Eigen::Vector2d& base_position, 
                                 const std::string& id = "first_estimate",
                                 float mean_base_height = 0.1) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        Eigen::Vector3d pos_3d(base_position.x(), base_position.y(), mean_base_height);
        auto marker = createBaseMarker(pos_3d, id, 1.0, 1.0, 0.0, true);  // Yellow, persistent
        
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

    void publishEstimatedBase(const Eigen::Vector2d& base_position, 
                             const std::string& id = "estimated_base",
                             float mean_base_height = 0.1) {
        visualization_msgs::msg::MarkerArray marker_array;
        
        Eigen::Vector3d pos_3d(base_position.x(), base_position.y(), mean_base_height);
        auto marker = createBaseMarker(pos_3d, id, 1.0, 0.0, 0.0, false);  // Red, 15s duration
        
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);
    }

private:
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    std::vector<BoundingBox> detections_;    
    std::chrono::steady_clock::time_point detection_last_update_;
    std::chrono::steady_clock::time_point valid_detection_last_update_;

    bool is_there_detection_{false};
    BoundingBox closest_bbox_;
    float min_distance_{0.0f};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int marker_id_counter_;


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
    
    visualization_msgs::msg::Marker createBaseMarker(const Eigen::Vector3d& position,
                                                    const std::string& ns,
                                                    float r, float g, float b, bool persistent = true) {
        visualization_msgs::msg::Marker marker;
        
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = ns;
        marker.id = marker_id_counter_++;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position (FRD to ENU conversion)
        marker.pose.position.x = position.y();
        marker.pose.position.y = position.x();
        marker.pose.position.z = -position.z();
        
        // Orientation (no rotation)
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        
        // Scale (1x1x0.1m)
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 0.1;
        
        // Color
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        
        // Different transparency for red estimated bases vs others
        if (r == 1.0 && g == 0.0 && b == 0.0) {
            marker.color.a = 0.4;  // Red estimated bases - more transparent
        } else {
            marker.color.a = 0.8;  // Semi-transparent for others
        }
        
        // Lifetime
        if (persistent) {
            marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // Persistent
        } else {
            marker.lifetime = rclcpp::Duration::from_nanoseconds(15e9);  // 15 seconds
        }
        
        return marker;
    }

    visualization_msgs::msg::Marker createCircleMarker(const Eigen::Vector3d& position,
                                                      const std::string& ns,
                                                      float radius,
                                                      float r, float g, float b, bool persistent = true) {
        visualization_msgs::msg::Marker marker;
        
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = ns;
        marker.id = marker_id_counter_++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Position (FRD to ENU conversion)
        marker.pose.position.x = position.y();
        marker.pose.position.y = position.x();
        marker.pose.position.z = -position.z();
        
        // Orientation (no rotation)
        marker.pose.orientation.w = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        
        // Scale (radius x radius x very thin height)
        marker.scale.x = radius * 2.0;  // Diameter
        marker.scale.y = radius * 2.0;  // Diameter  
        marker.scale.z = 0.01;          // Very thin cylinder (like a circle)
        
        // Color (more transparent for the circle)
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.3;  // More transparent than the square
        
        // Lifetime
        if (persistent) {
            marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // Persistent
        } else {
            marker.lifetime = rclcpp::Duration::from_nanoseconds(15e9);  // 15 seconds
        }
        
        return marker;
    }

};
