#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include "Base.hpp"

struct BoundingBox {
    float center_x;
    float center_y;
    float width;
    float height;
    float rotation; // radians, read from Detection2D.bbox.center.theta
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

        this->declare_parameter<double>("timeout", 10.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());

        // Camera setup moved to helper for clarity
        setupCameraParameters();

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
                    bbox.rotation = detection.bbox.center.theta;  // store rotation
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
                            const Eigen::Vector3d& position,
                            float confidence = 1.0f,
                            uint32_t detection_id = 0) 
    {
        auto msg = custom_msgs::msg::BaseDetection();
        msg.header.stamp = this->get_clock()->now();
        msg.position.x = position.x();
        msg.position.y = position.y(); 
        msg.position.z = position.z();
        msg.base_type = base_type;
        msg.confidence = confidence;
        msg.detection_id = detection_id;
        
        base_detection_pub_->publish(msg);
    }

    // Approximate backprojection: intersect camera ray with plane z = mean_base_height
    Eigen::Vector3d getApproximateBase(const Eigen::Vector3d &drone_pos,
                                       const Eigen::Vector3d &drone_orientation, // roll,pitch,yaw radians
                                       const BoundingBox &bbox,
                                       float mean_base_height)
    {
        double bbox_x = bbox.center_x;
        double bbox_y = bbox.center_y;

        // Assuming base is at mean_base_height above the ground
        // Converting to positive height value
        double height = - (drone_pos.z() - mean_base_height);

        // height_to_ground: ratio between distance seen in image (from left to right) and distance from the ground (height).
        double k = std::atan(1.1 / 2);

        // Yolo coordinates: x -> left to right, y -> top to bottom
        double x_img = height * std::tan(k * 2 * (bbox_x - 0.5));
        double y_img = height * std::tan(k * 2 * (bbox_y - 0.5));

        // Drone coordinates: x -> front, y -> right
        double x_drone = - y_img;
        double y_drone = x_img;

        // FRD (Forward-Right-Down) coordinates
        float yaw = drone_orientation.z();
        double frd_x = x_drone * cos(yaw) - y_drone * sin(yaw);
        double frd_y = x_drone * sin(yaw) + y_drone * cos(yaw);

        float base_x = drone_pos.x() + frd_x;
        float base_y = drone_pos.y() + frd_y;
        
        return Eigen::Vector3d({base_x, base_y, mean_base_height});
    }

    // Accurate estimation using solvePnP on rotated bbox corners (returns world 3D point of bbox center)
    Eigen::Vector3d getAccurateBase(const Eigen::Vector3d &drone_pos,
                                    const Eigen::Vector3d &drone_orientation, // roll,pitch,yaw
                                    const BoundingBox &bbox,
                                    float mean_base_height)
    {
        // Ensure we have rotation and size info
        // Compute pixel-space rotated rect
        double cx_px = bbox.center_x * static_cast<double>(image_width_);
        double cy_px = bbox.center_y * static_cast<double>(image_height_);
        double sz_w_px = bbox.width * static_cast<double>(image_width_);
        double sz_h_px = bbox.height * static_cast<double>(image_height_);
        double angle_deg = static_cast<double>(bbox.rotation) * (180.0 / M_PI);

        if (sz_w_px <= 1.0 || sz_h_px <= 1.0) {
            // degenerate detection, fallback
            return getApproximateBase(drone_pos, drone_orientation, bbox, mean_base_height);
        }

        cv::RotatedRect rrect(cv::Point2f(static_cast<float>(cx_px), static_cast<float>(cy_px)),
                              cv::Size2f(static_cast<float>(sz_w_px), static_cast<float>(sz_h_px)),
                              static_cast<float>(angle_deg));
        cv::Point2f box_pts_cv[4];
        rrect.points(box_pts_cv);

        std::vector<cv::Point2f> imagePoints;
        for (int i = 0; i < 4; ++i) imagePoints.emplace_back(box_pts_cv[i]);

        // Define object points in object frame (square on ground z=0), ordering must match imagePoints
        double s = bbox_real_size_;
        std::vector<cv::Point3f> objectPoints;
        objectPoints.emplace_back(cv::Point3f(-s/2.0f, -s/2.0f, 0.0f));
        objectPoints.emplace_back(cv::Point3f( s/2.0f, -s/2.0f, 0.0f));
        objectPoints.emplace_back(cv::Point3f( s/2.0f,  s/2.0f, 0.0f));
        objectPoints.emplace_back(cv::Point3f(-s/2.0f,  s/2.0f, 0.0f));

        cv::Mat rvec, tvec;
        bool ok = cv::solvePnP(objectPoints, imagePoints, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

        if (!ok || tvec.empty()) {
            // fallback to approximate if PnP failed
            return getApproximateBase(drone_pos, drone_orientation, bbox, mean_base_height);
        }

        // Convert tvec to Eigen (this is object center in camera coordinates)
        cv::Mat tvec_d;
        tvec.convertTo(tvec_d, CV_64F);
        Eigen::Vector3d obj_in_cam(tvec_d.at<double>(0), tvec_d.at<double>(1), tvec_d.at<double>(2));

        // Build transforms like in approximate method
        Eigen::Matrix3d R_world_drone = eulerToRot(drone_orientation.x(), drone_orientation.y(), drone_orientation.z());
        Eigen::Matrix3d R_drone_camera = eulerToRot(cam_roll_, cam_pitch_, cam_yaw_);
        Eigen::Matrix3d R_world_camera = R_world_drone * R_drone_camera;
        Eigen::Vector3d cam_in_drone(cam_tx_, cam_ty_, cam_tz_);
        Eigen::Vector3d cam_world = drone_pos + R_world_drone * cam_in_drone;

        // object in world = cam_world + R_world_camera * obj_in_cam
        Eigen::Vector3d obj_world = cam_world + R_world_camera * obj_in_cam;

        // If resulting Z is far from mean_base_height, we may want to project onto ground plane to keep z consistent
        // Prefer returning the 3D point from PnP; user can adjust z if they want an exact ground-plane value.
        return obj_world;
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

    // Camera params (members)
    double camera_fx_{500.0}, camera_fy_{500.0}, camera_cx_{320.0}, camera_cy_{240.0};
    int image_width_{640}, image_height_{480};
    double cam_roll_{0.0}, cam_pitch_{0.0}, cam_yaw_{0.0};
    double cam_tx_{0.0}, cam_ty_{0.0}, cam_tz_{0.0};
    double bbox_real_size_{1.0};

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // helper: build rotation matrix from roll,pitch,yaw (r,p,y) -> R (body/camera/world)
    Eigen::Matrix3d eulerToRot(double roll, double pitch, double yaw) {
        Eigen::AngleAxisd a_yaw(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd a_pitch(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd a_roll(roll, Eigen::Vector3d::UnitX());
        return (a_yaw * a_pitch * a_roll).toRotationMatrix();
    }

    // Setup camera intrinsics and distortion based on parameters (moved out from constructor)
    void setupCameraParameters() {
        // Camera geometry parameters (defaults will be overridden from params file)
        this->declare_parameter<double>("camera_horizontal_fov", 1.047); // ~60deg
        this->declare_parameter<int>("camera_original_width", 1920);
        this->declare_parameter<int>("camera_original_height", 1080);
        this->declare_parameter<int>("image_width", 800);   // published width (after crop+resize)
        this->declare_parameter<int>("image_height", 800);  // published height

        // Camera extrinsics: camera pose relative to vehicle body (roll,pitch,yaw in radians)
        this->declare_parameter<double>("camera_roll", 0.0);
        this->declare_parameter<double>("camera_pitch", 0.0);
        this->declare_parameter<double>("camera_yaw", 0.0);

        // Camera translation (camera position in body frame) in meters
        this->declare_parameter<double>("camera_tx", 0.0);
        this->declare_parameter<double>("camera_ty", 0.0);
        this->declare_parameter<double>("camera_tz", 0.0);

        // Distortion coefficients (k1,k2,p1,p2,k3)
        this->declare_parameter<double>("dist_k1", 0.0);
        this->declare_parameter<double>("dist_k2", 0.0);
        this->declare_parameter<double>("dist_p1", 0.0);
        this->declare_parameter<double>("dist_p2", 0.0);
        this->declare_parameter<double>("dist_k3", 0.0);

        // Real world bbox size (meters) used by accurate estimation
        this->declare_parameter<double>("bbox_real_size", 1.0);

        // load parameters to members
        double horizontal_fov_ = this->get_parameter("camera_horizontal_fov").as_double();
        int original_width_ = this->get_parameter("camera_original_width").as_int();
        int original_height_ = this->get_parameter("camera_original_height").as_int();
        image_width_ = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();

        cam_roll_ = this->get_parameter("camera_roll").as_double();
        cam_pitch_ = this->get_parameter("camera_pitch").as_double();
        cam_yaw_ = this->get_parameter("camera_yaw").as_double();
        cam_tx_ = this->get_parameter("camera_tx").as_double();
        cam_ty_ = this->get_parameter("camera_ty").as_double();
        cam_tz_ = this->get_parameter("camera_tz").as_double();

        bbox_real_size_ = this->get_parameter("bbox_real_size").as_double();

        // distortion
        double k1 = this->get_parameter("dist_k1").as_double();
        double k2 = this->get_parameter("dist_k2").as_double();
        double p1 = this->get_parameter("dist_p1").as_double();
        double p2 = this->get_parameter("dist_p2").as_double();
        double k3 = this->get_parameter("dist_k3").as_double();

        // Build intrinsics from horizontal_fov and original resolution
        double fx_orig = static_cast<double>(original_width_) / (2.0 * std::tan(horizontal_fov_ / 2.0));
        double fy_orig = fx_orig;
        double cx_orig = static_cast<double>(original_width_) / 2.0;
        double cy_orig = static_cast<double>(original_height_) / 2.0;

        // Account for central crop performed by publisher (publisher crops to square using min dimension)
        int crop_size = std::min(original_width_, original_height_);
        double crop_start_x = 0.0;
        double crop_start_y = 0.0;
        if (original_width_ > original_height_) {
            crop_start_x = (original_width_ - original_height_) / 2.0;
            crop_start_y = 0.0;
        } else if (original_height_ > original_width_) {
            crop_start_x = 0.0;
            crop_start_y = (original_height_ - original_width_) / 2.0;
        }

        // principal point in cropped image coords
        double cx_cropped = cx_orig - crop_start_x;
        double cy_cropped = cy_orig - crop_start_y;
        double cropped_size = static_cast<double>(crop_size);

        // Scale intrinsics to published (resized) image
        double scale = static_cast<double>(image_width_) / cropped_size; // assume square published
        camera_fx_ = fx_orig * scale;
        camera_fy_ = fy_orig * scale;
        camera_cx_ = cx_cropped * scale;
        camera_cy_ = cy_cropped * scale;

        // Fill cv::Mat camera matrix and distortion vector
        camera_matrix_ = (cv::Mat_<double>(3,3) << camera_fx_, 0.0, camera_cx_,
                                                  0.0, camera_fy_, camera_cy_,
                                                  0.0, 0.0, 1.0);
        dist_coeffs_ = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    }

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
