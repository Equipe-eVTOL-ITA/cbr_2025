#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include "drone/Drone.hpp"
#include "transformations.hpp"

// Utility function to normalize yaw error to [-π, π]
float normalizeYawError(float yaw_error) {
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;
    return yaw_error;
}

// mudar essa implementacao que move o drone usando velocidade
// usar posicao alvo e little goal

void move_local_by_speed(std::shared_ptr<Drone> drone, Eigen::Vector3d direction, float speed) {
    if (drone == nullptr) return;

    Eigen::Vector3d local_velocity = direction * speed;
    Eigen::Vector3d adjusted_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);
    drone->setLocalVelocity(adjusted_velocity.x(), adjusted_velocity.y(), adjusted_velocity.z(), 0.0f);
}

void move_local_by_speed(std::shared_ptr<Drone> drone, float vx, float vy, float vz) {
    if (drone == nullptr) return;

    Eigen::Vector3d local_velocity(vx, vy, vz);
    Eigen::Vector3d adjusted_velocity = adjust_velocity_using_yaw(local_velocity, drone->getOrientation()[2]);
    drone->setLocalVelocity(adjusted_velocity.x(), adjusted_velocity.y(), adjusted_velocity.z(), 0.0f);
}

bool move_local_by_waypoint(std::shared_ptr<Drone> drone, Eigen::Vector3d waypoint, float speed, float tolerance = 0.1f) {
    Eigen::Vector3d local_position = drone->getLocalPosition();

    Eigen::Vector3d diff = waypoint - local_position;
    if (diff.norm() < tolerance) return true;  // Close enough to the waypoint

    Eigen::Vector3d little_goal =
        local_position + (diff.norm() > speed ? diff.normalized() * speed : diff);

    drone->setLocalPosition(
        little_goal.x(),
        little_goal.y(),
        little_goal.z(),
        drone->getOrientation()[2]
    );

    return false;
}

void rotateYaw(std::shared_ptr<Drone> drone, float target_yaw, float yaw_rate = 0.3f, float tolerance = 0.05f) {
    if (drone == nullptr) return;

    float current_yaw = drone->getOrientation()[2];
    float yaw_diff = normalizeYawError(target_yaw - current_yaw);

    if (std::abs(yaw_diff) < tolerance) {
        drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f); // Stop rotation
        return;
    }

    float applied_yaw_rate = (yaw_diff > 0 ? yaw_rate : -yaw_rate);
    drone->setLocalVelocity(0.0f, 0.0f, 0.0f, applied_yaw_rate);
}