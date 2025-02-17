#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "PidController.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <array>

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State(), x_pid(0.9, 0.0, 0.05, 0.5), y_pid(0.9, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: VISIT BASES");

        float takeoff_height = *blackboard.get<float>("takeoff_height");
        const Eigen::Vector2d approx_base = *blackboard.get<Eigen::Vector2d>("approximate_base");
        initial_yaw = *blackboard.get<float>("initial_yaw");

        approx_goal = Eigen::Vector3d(approx_base.x(), approx_base.y(), takeoff_height);
        
        drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        if (!at_approximate_base){
            pos = drone->getLocalPosition();

            if ((approx_goal - pos).norm() < 0.10){
                drone->log("Arrived at estimated base.");
                at_approximate_base = true;
            }

            Eigen::Vector3d goal_diff = approx_goal - pos;
            if (goal_diff.norm() > max_velocity){
                goal_diff = goal_diff.normalized() * max_velocity;
            }
            little_goal = goal_diff + pos;

            drone->setLocalPosition(little_goal[0], little_goal[1], little_goal[2], initial_yaw);

            return "";
        }

        bboxes = drone->getBoundingBox();

        // Find the bounding box closest to the center
        double min_distance = std::numeric_limits<double>::max();
        if (!bboxes.empty()) {
            updateBBoxesBuffer(bboxes[0].center_x);
            for (const auto &bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }
            }
        }
        else{
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            updateBBoxesBuffer(0.0);
            return "";
        }

        //BBox detection is the same for the last 5s
        if (allElementsEqual(bboxes_buffer))
        {
            drone->log("Last 5s had no detections!");
            return "LOST BASE";
        }
        //Good to do PID Control
        else
        {
            x_rate = x_pid.compute(closest_bbox.center_y);
            y_rate = y_pid.compute(closest_bbox.center_x);
            drone->setLocalVelocity(x_rate, -y_rate, 0.0);
        }

        if (min_distance < 0.03){
            return "ARRIVED AT BASE";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        bboxes_buffer.clear();
        at_approximate_base = false;
    }

private:
    Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});
    Drone* drone;
    PidController x_pid, y_pid;
    float x_rate = 0.0, y_rate = 0.0;
    std::vector<DronePX4::BoundingBox> bboxes;
    DronePX4::BoundingBox closest_bbox;
    std::deque<float> bboxes_buffer;
    bool at_approximate_base = false;
    Eigen::Vector3d pos, approx_goal, little_goal;
    float max_velocity = 0.9;
    float initial_yaw;

    void updateBBoxesBuffer(float new_center_x)
    {
        if (bboxes_buffer.size() >= 100){
            bboxes_buffer.pop_front();
        }
        bboxes_buffer.push_back(new_center_x);
    }

    bool allElementsEqual(const std::deque<float>& buffer)
    {
        if (buffer.empty()) return true;
        float first_value = buffer[0];
        for (const auto& value : buffer){
            if (value != first_value) {
                return false;
            }
        }
        return buffer.size() == 100; // 5 seconds
    }

};