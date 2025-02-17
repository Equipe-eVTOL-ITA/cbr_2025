#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "PidController.hpp"

#include <deque>
#include <iostream>
#include <string>
#include <array>

class VisitBasesState : public fsm::State {
public:
    VisitBasesState() : fsm::State(), x_pid(0.23 * 0.15, 0.0, 0.05, 0.5), z_pid(0.6, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: READ QR CODES");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        //if (num_detections >4){
            
        //}


        bboxes = drone->getBarCodeLocation();

        // Find the bounding box closest to the center
        double min_distance = std::numeric_limits<double>::max();
        if (!bboxes.empty()) {
            updateBBoxesBuffer(bboxes[0].x());
            for (const auto &bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.x(), bbox.y()) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }
            }
        } else {
            drone->setLocalVelocity(0.0, go_left ? 0.3 : -0.3, 0.0);
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
            float area = closest_bbox.z() * closest_bbox.w();
            
            x_rate = x_pid.compute(area);
            z_rate = z_pid.compute(closest_bbox.y());
            drone->setLocalVelocity(x_rate, go_left ? 0.3 : -0.3, -z_rate, 0.0);
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        bboxes_buffer.clear();
        
    }

private:
    Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});
    Drone* drone;
    PidController x_pid, z_pid;
    float x_rate = 0.0, z_rate = 0.0;
    std::vector<Eigen::Vector4d> bboxes;
    Eigen::Vector4d closest_bbox;
    std::deque<float> bboxes_buffer;
    bool at_approximate_base = false;
    Eigen::Vector3d pos, approx_goal, little_goal;
    float max_velocity = 0.9;
    bool go_left = false;

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