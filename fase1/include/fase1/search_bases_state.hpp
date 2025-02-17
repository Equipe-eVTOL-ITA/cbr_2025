#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"

class SearchBasesState : public fsm::State {
public:
    SearchBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("STATE: SEARCH BASES");

        initial_yaw = *blackboard.get<float>("initial_yaw");
        waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        bases = blackboard.get<std::vector<Base>>("bases");
        //Eigen::Vector3d* last_search_ptr = blackboard.get<Eigen::Vector3d>("last search position");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        
        pos = drone->getLocalPosition();
        goal_ptr = getNextPoint(waypoints);

        if (goal_ptr == nullptr)
            return "SEARCH ENDED";

        if ((pos - goal_ptr->coordinates).norm() < 0.15){
            goal_ptr->is_visited = true;
            drone->log("Waypoint visited.");
        }

        goal = goal_ptr->coordinates;
        goal_diff = goal-pos;
        if (goal_diff.norm() > max_velocity){
            goal_diff = goal_diff.normalized() * max_velocity;
        }
        goal = goal_diff + pos;
        drone->setLocalPosition(goal[0], goal[1], goal[2], initial_yaw);

        bboxes = drone->getBoundingBox();

        if (!bboxes.empty() && !previous_bboxes.empty()) {
            if (bboxes[0].center_x != previous_bboxes[0].center_x){
                for (const auto& bbox : bboxes) {
                    
                    const Eigen::Vector2d offset = getApproximateBase(bbox.center_x, bbox.center_y);
                    const Eigen::Vector2d approx_base = pos.head<2>() + offset;
                    drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");

                    bool is_known_base = false;
                    for (const auto& base : *bases) {
                        float horizontal_distance = (approx_base - base.coordinates.head<2>()).norm();
                        drone->log("Dist " + std::to_string(horizontal_distance) + " to {" 
                                    + std::to_string(base.coordinates[0]) + ", " + std::to_string(base.coordinates[1]) + "}");
                        if (horizontal_distance < 1.7) {
                            drone->log("Known base!");
                            is_known_base = true;
                            break;
                        }
                    }

                    if (!is_known_base) {
                        blackboard.set<Eigen::Vector2d>("approximate_base", approx_base);
                        return "BASES FOUND";
                    }
                }
            }

        }
        previous_bboxes = bboxes;
        return ""; 
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        
        pos = drone->getLocalPosition();
        blackboard.set<Eigen::Vector3d>("last search position", pos);
    }
    
private:
    Drone* drone;
    std::vector<Base>* bases;
    std::vector<DronePX4::BoundingBox> bboxes, previous_bboxes;
    std::vector<ArenaPoint>* waypoints;
    ArenaPoint* goal_ptr;
    Eigen::Vector3d pos, goal, goal_diff;
    const float max_velocity = 0.4;
    float initial_yaw;
    
    Eigen::Vector2d getApproximateBase(double x, double y) {
        double x_max_dist = 2.3; //distance on ground from left to right of picture when flying at takeoff altitude
        double y_max_dist = x_max_dist * 3 / 4; //based on 4:3 image proportion - from up to down

        double avg_hgt = 2.5; // Averaged distance to landing pad

        //Considering that pixel dimensions are like angular resolution
        double k_x = std::atan((x_max_dist / 2) / avg_hgt);
        double k_y = std::atan((y_max_dist / 2) / avg_hgt);

        double distance_x = avg_hgt * std::tan(k_x * 2 * (x-0.5));
        double distance_y = - avg_hgt * std::tan(k_y * 2 * (y-0.5)); //Minus sign comes from yolo assuming 1 is on top of image
        
        return Eigen::Vector2d({distance_y, distance_x});
    }

    ArenaPoint* getNextPoint(std::vector<ArenaPoint>* waypoints) {
        for (auto& point : *waypoints) {
            if (!point.is_visited) {
                return &point;
            }
        }
        
        // Return nullptr if no unvisited points are found
        return nullptr;
    }
};