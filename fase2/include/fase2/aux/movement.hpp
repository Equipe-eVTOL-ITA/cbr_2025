#pragma once

#include <Eigen/Eigen>
#include "drone/Drone.hpp"
#include "transformations.hpp"

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