#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "PidController.hpp"

#include <iostream>
#include <string>

class CentralizeBaseState : public fsm::State {
public:
    CentralizeBaseState() : fsm::State(), x_pid(0.9, 0.0, 0.05, 0.5), y_pid(0.9, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: CENTRALIZE BASE");

        start_time_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();

        if (elapsed_time > 8) {
            drone->log("Exceeded 8s with no detections");
            return "CENTRALIZED BASE";
        }

        bboxes = drone->getBoundingBox();

        // Find the bounding box closest to the center
        double min_distance = std::numeric_limits<double>::max();
        if (!bboxes.empty() && bboxes[0].center_x != 0.0) {
            start_time_ = std::chrono::steady_clock::now();
            for (const auto &bbox : bboxes) {
                double distance = (Eigen::Vector2d(bbox.center_x, bbox.center_y) - image_center).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_bbox = bbox;
                }
            }
            x_rate = x_pid.compute(closest_bbox.center_y);
            y_rate = y_pid.compute(closest_bbox.center_x);
            drone->setLocalVelocity(x_rate, -y_rate, 0.0);
        }
        else{
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            return "";
        }


        if (min_distance < 0.03){
            return "CENTRALIZED BASE";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        x_rate = 0.0;
        y_rate = 0.0;
    }

private:
    Drone* drone;
    std::vector<DronePX4::BoundingBox> bboxes;
    DronePX4::BoundingBox closest_bbox;

    Eigen::Vector2d image_center = Eigen::Vector2d({0.5, 0.5});
    PidController x_pid, y_pid;
    float x_rate = 0.0, y_rate = 0.0;

    std::chrono::steady_clock::time_point start_time_;

};