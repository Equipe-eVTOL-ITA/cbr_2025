#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "Base.hpp"
#include "ArenaPoint.hpp"
#include "drone/Drone.hpp"
#include "vision_fase2.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(this->drone == nullptr || this->vision == nullptr) return;
        drone->log("STATE: SEARCH BASE");

        this->waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        this->bases = blackboard.get<std::vector<Base>>("bases");

        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->initial_yaw = drone->getOrientation()[2];
        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->known_base_radius = *blackboard.get<float>("known_base_radius");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        this->pos = this->drone->getLocalPosition();
        this->yaw = this->drone->getOrientation()[2];

        if (this->vision->isThereDetection()) {
            auto bboxes = this->vision->getDetections();

            for (const auto& bbox : bboxes) {
                const Eigen::Vector2d approx_base = getApproximateBase(bbox);
                drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");

                bool is_known_base = false;
                for (const auto& base : *this->bases) {
                    float horizontal_distance = (approx_base - base.coordinates.head<2>()).norm();
                    drone->log("Dist " + std::to_string(horizontal_distance) + " to {" 
                                + std::to_string(base.coordinates[0]) + ", " + std::to_string(base.coordinates[1]) + "}");
                    if (horizontal_distance < this->known_base_radius) {
                        drone->log("Known base!");
                        is_known_base = true;
                        break;
                    }
                }

                if (!is_known_base) {
                    blackboard.set<Eigen::Vector2d>("approximate_base", approx_base);
                    return "BASE FOUND";
                }
            }                

        }

        auto goal_point = this->getNextPoint(this->waypoints);
        if(goal_point == nullptr) return "SEARCH ENDED";
        
        Eigen::Vector3d goal = goal_point->coordinates;
        Eigen::Vector3d goal_diff = goal - this->pos;

        if (goal_diff.norm() < this->position_tolerance) {
            goal_point->is_visited = true;
            this->drone->log("Ponto visitado: {" + std::to_string(pos.x()) + ", "
                            + std::to_string(pos.y()) + ", " + std::to_string(pos.z()) + "}");
            return ""; 
        }

        Eigen::Vector3d little_goal = this->pos + (goal_diff.norm() > max_velocity ? goal_diff.normalized() * max_velocity : goal_diff);

        this->drone->setLocalPosition(
            little_goal[0],
            little_goal[1],
            little_goal[2],
            this->initial_yaw
        );

        return "";

    }

    void on_exit(fsm::Blackboard &blackboard) override {
        this->pos = this->drone->getLocalPosition();
        blackboard.set<Eigen::Vector3d>("last search position", this->pos);
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    std::vector<ArenaPoint>* waypoints;
    std::vector<Base>* bases;

    float initial_yaw;
    float yaw;
    float position_tolerance;
    float max_velocity;

    float known_base_radius;
    float height_to_ground;

    Eigen::Vector3d pos;
    
    ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
        if (!waypoints) return nullptr;
        for (auto& point : *waypoints) {
            if (!point.is_visited) {
                return &point;
            }
        }
        // Return nullptr if no unvisited points are found
        return nullptr;
    }

    Eigen::Vector2d getApproximateBase(BoundingBox bbox) {

        double bbox_x = bbox.center_x;
        double bbox_y = bbox.center_y;

        // Assuming base is at 0.75m above the ground
        double height = -this->pos.z() - 0.75;

        // height_to_ground: distance seen in image (from left to right) when camera is at 1m distance from the ground.
        double k = std::atan(this->height_to_ground / 2);

        // Yolo coordinates: x -> left to right, y -> top to bottom
        double x_img = height * std::tan(k * 2 * (bbox_x - 0.5));
        double y_img = height * std::tan(k * 2 * (bbox_y - 0.5));

        // Drone coordinates: x -> front, y -> right
        double x_drone = - y_img;
        double y_drone = x_img;

        // FRD (Forward-Right-Down) coordinates
        double frd_x = x_drone * cos(this->yaw) - y_drone * sin(this->yaw);
        double frd_y = x_drone * sin(this->yaw) + y_drone * cos(this->yaw);
        
        return this->pos.head<2>() + Eigen::Vector2d({frd_x, frd_y});
    }
};