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

struct BoundingBox {
    float center_x;   // May be normalized [0..1] or pixels
    float center_y;   // May be normalized [0..1] or pixels
    float width;      // same units as center (normalized or px)
    float height;     // same units as center (normalized or px)
    float rotation;   // radians, Detection2D.bbox.center.theta (CCW in image frame)
    float confidence;
    std::string class_id;
    int64_t timestamp;
};

class VisionNode : public rclcpp::Node {
public:
    VisionNode() : Node("fase1_vision") {
        // QoS for vision
        rclcpp::QoS vision_qos(10);
        vision_qos.best_effort();
        vision_qos.durability(rclcpp::DurabilityPolicy::Volatile);

        this->declare_parameter<double>("timeout", 10.0);
        timeout_ = std::chrono::duration<double>(this->get_parameter("timeout").as_double());

        loadCameraAndModelParameters_();

        detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
            "/vertical_camera/classification",
            vision_qos,
            [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
                detections_.clear();
                detection_last_update_ = std::chrono::steady_clock::now();

                if (msg->detections.empty()) {
                    is_there_detection_ = false;
                    return;
                }

                is_there_detection_ = true;
                valid_detection_last_update_ = std::chrono::steady_clock::now();

                for (const auto& d : msg->detections) {
                    BoundingBox b;
                    b.center_x = d.bbox.center.position.x;
                    b.center_y = d.bbox.center.position.y;
                    b.width    = d.bbox.size_x;
                    b.height   = d.bbox.size_y;
                    b.rotation = d.bbox.center.theta;
                    if (!d.results.empty()) {
                        b.confidence = d.results[0].hypothesis.score;
                        b.class_id   = d.results[0].hypothesis.class_id;
                    } else {
                        b.confidence = 0.f;
                        b.class_id   = "";
                    }
                    b.timestamp = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
                    detections_.push_back(b);
                }

                computeClosestBbox_();
            }
        );

        base_detection_pub_ = this->create_publisher<custom_msgs::msg::BaseDetection>("/telemetry/bases", 10);

        RCLCPP_INFO(this->get_logger(), "Vision node initialized. image=%dx%d, fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                    image_width_, image_height_, camera_fx_, camera_fy_, camera_cx_, camera_cy_);
    }

    // Timers for FSM
    double lastDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - detection_last_update_).count();
    }
    double lastBaseDetectionTime() {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - valid_detection_last_update_).count();
    }

    bool isThereDetection() {
        if (lastDetectionTime() > timeout_.count()) return false;
        return is_there_detection_;
    }

    BoundingBox getClosestBbox() const { return closest_bbox_; }
    float getMinDistance() const { return min_distance_; }

    std::vector<BoundingBox> getDetections() const { return detections_; }

    void publishBaseDetection(const std::string& base_type,
                              const Eigen::Vector3d& position,
                              float confidence = 1.0f,
                              uint32_t detection_id = 0) 
    {
        custom_msgs::msg::BaseDetection msg;
        msg.header.stamp = this->get_clock()->now();
        msg.position.x = position.x();
        msg.position.y = position.y();
        msg.position.z = position.z();
        msg.base_type  = base_type;
        msg.confidence = confidence;
        msg.detection_id = detection_id;
        base_detection_pub_->publish(msg);
    }

    // === Geometric helpers ===================================================

    // Intersect the pixel ray with plane z = mean_base_height (world FRD, z positive down)
    Eigen::Vector3d getApproximateBase(const Eigen::Vector3d &drone_pos_w,
                                       const Eigen::Vector3d &drone_rpy_w,   // roll,pitch,yaw (rad) in FRD
                                       const BoundingBox &bbox,
                                       float mean_base_height /*z-plane*/)
    {
        // 1) Pixel coordinates (handle normalized or pixel input)
        auto [u, v] = bboxCenterInPixels_(bbox);

        // 2) Get normalized ray in camera frame (OpenCV convention)
        Eigen::Vector3d ray_cam = pixelToCamRay_(u, v);  // (x,y,1) normalized

        // 3) Build world↤body and body↤camera transforms
        const Eigen::Matrix3d R_w_b = rpyToMatrixFRD_(drone_rpy_w.x(), drone_rpy_w.y(), drone_rpy_w.z());
        const Eigen::Matrix3d R_b_c = rpyToMatrixFRD_(cam_roll_body_, cam_pitch_body_, cam_yaw_body_); // maps camera→body
        const Eigen::Matrix3d R_w_c = R_w_b * R_b_c;

        // 4) Camera origin in world coordinates
        const Eigen::Vector3d t_b_c(cam_tx_, cam_ty_, cam_tz_);           // camera origin expressed in body
        const Eigen::Vector3d cam_world = drone_pos_w + R_w_b * t_b_c;    // world

        // 5) Ray in world
        const Eigen::Vector3d ray_world = R_w_c * ray_cam;

        const double denom = ray_world.z();
        if (std::abs(denom) < 1e-8) {
            // Ray nearly parallel to plane -> fallback to camera XY projection at plane height
            Eigen::Vector3d fallback = cam_world;
            fallback.z() = mean_base_height;
            return fallback;
        }

        const double t = (static_cast<double>(mean_base_height) - cam_world.z()) / denom;
        if (t < 0.0) {
            // Intersection is behind camera; clamp to a reasonable far point on the plane
            Eigen::Vector3d far_pt = cam_world + (100.0) * ray_world.normalized();
            far_pt.z() = mean_base_height;
            return far_pt;
        }

        return cam_world + t * ray_world;
    }

    // Pose-from-rectangle using PnP (expects bbox to be the minimal-area oriented rectangle of a 1x1 m square on the ground)
    // Returns center of the square in WORLD frame.
    Eigen::Vector3d getAccurateBase(const Eigen::Vector3d &drone_pos_w,
                                    const Eigen::Vector3d &drone_rpy_w,
                                    const BoundingBox &bbox)
    {
        // Early out: degenerate bbox → fallback
        auto [cx_px, cy_px] = bboxCenterInPixels_(bbox);
        auto [w_px, h_px]   = bboxSizeInPixels_(bbox);
        if (w_px <= 2.0 || h_px <= 2.0) {
            // ill-conditioned
            return getApproximateBase(drone_pos_w, drone_rpy_w, bbox, default_ground_z_);
        }

        // Build a rotated rectangle → 4 image points (float)
        const float angle_deg = static_cast<float>(bbox.rotation * 180.0 / M_PI);
        cv::RotatedRect rrect(cv::Point2f(static_cast<float>(cx_px), static_cast<float>(cy_px)),
                              cv::Size2f(static_cast<float>(w_px), static_cast<float>(h_px)),
                              angle_deg);

        cv::Point2f corners[4];
        rrect.points(corners);

        // Order image points consistently: top-left, top-right, bottom-right, bottom-left
        std::vector<cv::Point2f> imgPts(4);
        orderRectCorners_(corners, imgPts);

        // Define object points (meters) for a 1x1 square on z=0 of the object's local frame (center at origin).
        const double S = bbox_real_size_;
        std::vector<cv::Point3f> objPts{
            {-static_cast<float>(S/2.0), -static_cast<float>(S/2.0), 0.f},  // TL
            { static_cast<float>(S/2.0), -static_cast<float>(S/2.0), 0.f},  // TR
            { static_cast<float>(S/2.0),  static_cast<float>(S/2.0), 0.f},  // BR
            {-static_cast<float>(S/2.0),  static_cast<float>(S/2.0), 0.f}   // BL
        };

        cv::Mat rvec, tvec;
        bool ok = false;

        // Prefer IPPE_SQUARE when available (robust for planar squares). Fallback to ITERATIVE.
#ifdef cv_SOLVEPNP_IPPE_SQUARE
        ok = cv::solvePnP(objPts, imgPts, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
#else
        ok = cv::solvePnP(objPts, imgPts, camera_matrix_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
#endif

        if (!ok || tvec.empty()) {
            return getApproximateBase(drone_pos_w, drone_rpy_w, bbox, default_ground_z_);
        }

        cv::Mat tvec64;
        tvec.convertTo(tvec64, CV_64F);
        Eigen::Vector3d obj_in_cam(tvec64.at<double>(0), tvec64.at<double>(1), tvec64.at<double>(2));

        const Eigen::Matrix3d R_w_b = rpyToMatrixFRD_(drone_rpy_w.x(), drone_rpy_w.y(), drone_rpy_w.z());
        const Eigen::Matrix3d R_b_c = rpyToMatrixFRD_(cam_roll_body_, cam_pitch_body_, cam_yaw_body_);
        const Eigen::Matrix3d R_w_c = R_w_b * R_b_c;
        const Eigen::Vector3d t_b_c(cam_tx_, cam_ty_, cam_tz_);
        const Eigen::Vector3d cam_world = drone_pos_w + R_w_b * t_b_c;

        Eigen::Vector3d obj_world = cam_world + R_w_c * obj_in_cam;

        // Optional: If user wants the point exactly on z=ground, project along world ray to z=known.
        if (lock_on_ground_plane_) {
            // Recompute ray from camera to object center direction:
            Eigen::Vector3d ray_world = (obj_world - cam_world);
            const double denom = ray_world.z();
            if (std::abs(denom) > 1e-8) {
                const double t = (ground_z_for_lock_ - cam_world.z()) / denom;
                obj_world = cam_world + t * ray_world;
            }
        }
        return obj_world;
    }

private:
    // ==== Members ============================================================
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
    std::vector<BoundingBox> detections_;
    std::chrono::steady_clock::time_point detection_last_update_{};
    std::chrono::steady_clock::time_point valid_detection_last_update_{};
    std::chrono::duration<double> timeout_{10.0};
    bool is_there_detection_{false};

    BoundingBox closest_bbox_{};
    float min_distance_{0.0f};

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<custom_msgs::msg::BaseDetection>::SharedPtr base_detection_pub_;

    // Camera intrinsics (for the PUBLISHED 800x800 image)
    double camera_fx_{600.0}, camera_fy_{600.0}, camera_cx_{400.0}, camera_cy_{400.0};
    int image_width_{800}, image_height_{800};

    // Undistortion
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Extrinsics: camera pose expressed in BODY (FRD) frame
    // R_b_c maps camera -> body; t_b_c is camera origin expressed in body (meters).
    double cam_roll_body_{0.0}, cam_pitch_body_{0.0}, cam_yaw_body_{0.0};
    double cam_tx_{0.0}, cam_ty_{0.0}, cam_tz_{0.0};

    // Real world base size (meters)
    double bbox_real_size_{1.0};

    // BBox input mode
    bool bbox_is_normalized_{true};

    // Optional: force the accurate estimate to lie on a plane
    bool lock_on_ground_plane_{false};
    double ground_z_for_lock_{0.0};   // if lock_on_ground_plane_=true, project to this z
    double default_ground_z_{0.0};     // used by approximate fallback if needed

    // ==== Setup & utilities ==================================================

    // Z-down FRD rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll) (right-handed; +yaw is clockwise seen from above)
    Eigen::Matrix3d rpyToMatrixFRD_(double roll, double pitch, double yaw) const {
        Eigen::AngleAxisd a_z(yaw,   Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd a_y(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd a_x(roll,  Eigen::Vector3d::UnitX());
        return (a_z * a_y * a_x).toRotationMatrix();
    }

    // Convert pixel (u,v) to normalized camera ray (x,y,1) using cv::undistortPoints
    Eigen::Vector3d pixelToCamRay_(double u_px, double v_px) const {
        cv::Mat src(1, 1, CV_64FC2);
        src.at<cv::Vec2d>(0,0)[0] = u_px;
        src.at<cv::Vec2d>(0,0)[1] = v_px;
        cv::Mat dst;
        cv::undistortPoints(src, dst, camera_matrix_, dist_coeffs_, cv::noArray(), cv::noArray());
        const cv::Vec2d p = dst.at<cv::Vec2d>(0,0);
        Eigen::Vector3d ray(p[0], p[1], 1.0);
        // Normalize (not strictly required, but keeps scale consistent)
        return ray.normalized();
    }

    // Convert bbox center and size to pixels (handles normalized [0..1] or pixel inputs)
    std::pair<double,double> bboxCenterInPixels_(const BoundingBox& b) const {
        if (bbox_is_normalized_) {
            return { static_cast<double>(b.center_x) * image_width_,
                     static_cast<double>(b.center_y) * image_height_ };
        } else {
            return { static_cast<double>(b.center_x), static_cast<double>(b.center_y) };
        }
    }
    std::pair<double,double> bboxSizeInPixels_(const BoundingBox& b) const {
        if (bbox_is_normalized_) {
            return { static_cast<double>(b.width) * image_width_,
                     static_cast<double>(b.height) * image_height_ };
        } else {
            return { static_cast<double>(b.width), static_cast<double>(b.height) };
        }
    }

    // Order 4 corners of a rotated rectangle into TL,TR,BR,BL in image coords (origin top-left, y down)
    void orderRectCorners_(const cv::Point2f in[4], std::vector<cv::Point2f>& out) const {
        std::vector<cv::Point2f> pts(in, in+4);

        // Sum and diff tricks to find TL/BR and TR/BL
        auto sum = [](const cv::Point2f& p){ return p.x + p.y; };
        auto diff= [](const cv::Point2f& p){ return p.x - p.y; };

        int idx_tl = 0, idx_br = 0;
        float min_sum = 1e9f, max_sum = -1e9f;
        for (int i=0; i<4; ++i) {
            float s = sum(pts[i]);
            if (s < min_sum) { min_sum = s; idx_tl = i; }
            if (s > max_sum) { max_sum = s; idx_br = i; }
        }

        int idx_tr = 0, idx_bl = 0;
        float max_diff = -1e9f, min_diff = 1e9f;
        for (int i=0; i<4; ++i) {
            if (i==idx_tl || i==idx_br) continue;
            float d = diff(pts[i]);
            if (d > max_diff) { max_diff = d; idx_tr = i; }
            if (d < min_diff) { min_diff = d; idx_bl = i; }
        }

        out[0] = pts[idx_tl];
        out[1] = pts[idx_tr];
        out[2] = pts[idx_br];
        out[3] = pts[idx_bl];
    }

    void computeClosestBbox_() {
        if (detections_.empty()) {
            is_there_detection_ = false;
            return;
        }

        is_there_detection_ = true;
        Eigen::Vector2d ic(0.5, 0.5);  // normalized image center
        float best = 1e9f;
        BoundingBox best_box{};
        for (const auto& b : detections_) {
            // Work in normalized space regardless of input mode
            double cx = bbox_is_normalized_ ? b.center_x : b.center_x / static_cast<double>(image_width_);
            double cy = bbox_is_normalized_ ? b.center_y : b.center_y / static_cast<double>(image_height_);
            double d = (Eigen::Vector2d(cx, cy) - ic).norm();
            if (d < best) { best = static_cast<float>(d); best_box = b; }
        }
        closest_bbox_ = best_box;
        min_distance_ = best;
    }

    void loadCameraAndModelParameters_() {
        // ===== General node params
        // These next two affect only fallback behavior in getAccurateBase and the approximate method:
        this->declare_parameter<double>("mean_base_height", 0.0);  // plane z for approximate
        this->declare_parameter<bool>("accurate_project_to_ground", false);
        this->declare_parameter<double>("accurate_ground_z", 0.0);
        default_ground_z_     = this->get_parameter("mean_base_height").as_double();
        lock_on_ground_plane_ = this->get_parameter("accurate_project_to_ground").as_bool();
        ground_z_for_lock_    = this->get_parameter("accurate_ground_z").as_double();

        // ===== BBox coordinate mode
        this->declare_parameter<bool>("bbox_is_normalized", true);
        bbox_is_normalized_ = this->get_parameter("bbox_is_normalized").as_bool();

        // ===== Camera MODEL (intrinsics) for the PUBLISHED image
        // Option A) Provide fx,fy,cx,cy directly (preferred if you calibrated the resized stream)
        this->declare_parameter<double>("fx", 0.0);
        this->declare_parameter<double>("fy", 0.0);
        this->declare_parameter<double>("cx", 0.0);
        this->declare_parameter<double>("cy", 0.0);

        // Option B) Build from FOV and raw sensor size + crop-to-square + resize
        this->declare_parameter<double>("camera_horizontal_fov", 1.047); // ~60°
        this->declare_parameter<int>("camera_original_width", 1920);
        this->declare_parameter<int>("camera_original_height", 1080);

        this->declare_parameter<int>("image_width", 800);
        this->declare_parameter<int>("image_height", 800);

        // Distortion (k1,k2,p1,p2,k3) — on the *published* image
        this->declare_parameter<double>("dist_k1", 0.0);
        this->declare_parameter<double>("dist_k2", 0.0);
        this->declare_parameter<double>("dist_p1", 0.0);
        this->declare_parameter<double>("dist_p2", 0.0);
        this->declare_parameter<double>("dist_k3", 0.0);

        // ===== Camera EXTRINSICS (pose of camera expressed in BODY/FRD frame)
        // R_b_c = Rz(yaw)*Ry(pitch)*Rx(roll), mapping CAMERA vectors → BODY (FRD)
        // Zero means the two frames are aligned (same axes).
        this->declare_parameter<double>("camera_roll", 0.0);
        this->declare_parameter<double>("camera_pitch", 0.0);
        this->declare_parameter<double>("camera_yaw", 0.0);
        this->declare_parameter<double>("camera_tx", 0.0);
        this->declare_parameter<double>("camera_ty", 0.0);
        this->declare_parameter<double>("camera_tz", 0.0);

        // Real-world base size in meters (for PnP)
        this->declare_parameter<double>("bbox_real_size", 1.0);

        // Fetch
        image_width_  = this->get_parameter("image_width").as_int();
        image_height_ = this->get_parameter("image_height").as_int();

        bbox_real_size_ = this->get_parameter("bbox_real_size").as_double();

        cam_roll_body_  = this->get_parameter("camera_roll").as_double();
        cam_pitch_body_ = this->get_parameter("camera_pitch").as_double();
        cam_yaw_body_   = this->get_parameter("camera_yaw").as_double();
        cam_tx_         = this->get_parameter("camera_tx").as_double();
        cam_ty_         = this->get_parameter("camera_ty").as_double();
        cam_tz_         = this->get_parameter("camera_tz").as_double();

        // Intrinsics selection
        const double fx_param = this->get_parameter("fx").as_double();
        const double fy_param = this->get_parameter("fy").as_double();
        const double cx_param = this->get_parameter("cx").as_double();
        const double cy_param = this->get_parameter("cy").as_double();

        if (fx_param > 0.0 && fy_param > 0.0) {
            camera_fx_ = fx_param;
            camera_fy_ = fy_param;
            camera_cx_ = (cx_param > 0.0) ? cx_param : (static_cast<double>(image_width_)  * 0.5);
            camera_cy_ = (cy_param > 0.0) ? cy_param : (static_cast<double>(image_height_) * 0.5);
        } else {
            // Build from FOV + crop-to-square + resize
            const double hfov     = this->get_parameter("camera_horizontal_fov").as_double();
            const int    W_orig   = this->get_parameter("camera_original_width").as_int();
            const int    H_orig   = this->get_parameter("camera_original_height").as_int();

            // Focal in px for the original (uncropped) width
            const double fx_orig = static_cast<double>(W_orig) / (2.0 * std::tan(hfov * 0.5));
            const double fy_orig = fx_orig;

            // Principal point at original center
            const double cx_orig = static_cast<double>(W_orig) * 0.5;
            const double cy_orig = static_cast<double>(H_orig) * 0.5;

            // Center crop to square of size min(W,H)
            const int crop = std::min(W_orig, H_orig);
            double crop_start_x = 0.0, crop_start_y = 0.0;
            if (W_orig > H_orig) crop_start_x = (W_orig - H_orig) * 0.5;
            if (H_orig > W_orig) crop_start_y = (H_orig - W_orig) * 0.5;

            const double cx_cropped = cx_orig - crop_start_x;
            const double cy_cropped = cy_orig - crop_start_y;

            const double scale = static_cast<double>(image_width_) / static_cast<double>(crop);
            camera_fx_ = fx_orig * scale;
            camera_fy_ = fy_orig * scale;
            camera_cx_ = cx_cropped * scale;
            camera_cy_ = cy_cropped * scale;
        }

        // Distortion on the *published* image coordinates
        const double k1 = this->get_parameter("dist_k1").as_double();
        const double k2 = this->get_parameter("dist_k2").as_double();
        const double p1 = this->get_parameter("dist_p1").as_double();
        const double p2 = this->get_parameter("dist_p2").as_double();
        const double k3 = this->get_parameter("dist_k3").as_double();

        camera_matrix_ = (cv::Mat_<double>(3,3) << camera_fx_, 0.0, camera_cx_,
                                                   0.0, camera_fy_, camera_cy_,
                                                   0.0, 0.0, 1.0);
        dist_coeffs_   = (cv::Mat_<double>(1,5) << k1, k2, p1, p2, k3);
    }
};
