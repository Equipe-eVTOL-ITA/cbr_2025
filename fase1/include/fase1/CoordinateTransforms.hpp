#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

class CoordinateTransforms {
public:
    CoordinateTransforms(
        const Eigen::Vector3d& t_dc,
        const Eigen::Matrix3d& R_dc,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        float ground_z,
        double initial_yaw)
        : t_dc_(t_dc),
          R_dc_(R_dc),
          camera_matrix_(camera_matrix),
          dist_coeffs_(dist_coeffs),
          ground_z_(ground_z),
          initial_yaw_(initial_yaw) {}

    // Transform image coordinates to world coordinates
    Eigen::Vector3d ImageToWorld(
        const Eigen::Vector3d& drone_pos,
        const Eigen::Vector3d& drone_orientation,
        float center_x,
        float center_y);

    // Transform world coordinates to image coordinates
    bool WorldToImage(
        const Eigen::Vector3d& drone_pos,
        const Eigen::Vector3d& drone_orientation,
        const Eigen::Vector3d& pad_pos,
        float& center_x,
        float& center_y,
        float& size_x,
        float& size_y,
        const float pad_size = 1.0f);

private:
    Eigen::Vector3d t_dc_;         // Translation from drone to camera frame
    Eigen::Matrix3d R_dc_;         // Rotation from drone to camera frame
    cv::Mat camera_matrix_;        // Camera intrinsic matrix
    cv::Mat dist_coeffs_;          // Distortion coefficients
    float ground_z_;               // Ground plane z-coordinate
    double initial_yaw_;           // Initial yaw angle

    // Helper function to compute rotation matrix from drone orientation
    Eigen::Matrix3d ComputeR_wd(const Eigen::Vector3d& drone_orientation);
};
