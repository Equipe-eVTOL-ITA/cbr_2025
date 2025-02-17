#include <Eigen/Eigen>
#include <vector>
#include <limits>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "Base.hpp"
#include "PidController.hpp"

#include <iostream>
#include <string>

class ReadState : public fsm::State {
public:
    ReadState() : fsm::State(), x_pid(0.9, 0.0, 0.05, 0.5), y_pid(0.9, 0.0, 0.05, 0.5)  {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        drone = blackboard.get<Drone>("drone");
        drone->log("STATE: READ BASE");

        // min height = 60cm above base height
        Base current_base = *blackboard.get<Base>("current_base");
        float takeoff_height = *blackboard.get<float>("takeoff_height");
        min_height = current_base.coordinates[2] - takeoff_height - 0.45;

        pos = drone->getLocalPosition();

        goal = Eigen::Vector3d({pos[0], pos[1], min_height});
        drone->log("Terminal position: {" + std::to_string(goal[0]) + ", " + std::to_string(goal[1]) + ", " + std::to_string(goal[2]) + "}");

        start_time_ = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
        if (elapsed_time > 5) {
            drone->log("Exceeded 5s with no detections");
            return "GOT QR CODE";
        }

        pos = drone->getLocalPosition();
        if (pos[2] >= min_height){
            drone->log("Reached terminal height.");
            return "GOT QR CODE";
        }

        std::string qrcode = drone->readQRCode();
        if (qrcode != ""){
            drone->log("QR CODE READS:: " + qrcode);
            return "GOT QR CODE";
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
            drone->setLocalVelocity(x_rate, -y_rate, 0.15, 0.0);
        }
        else{
            drone->setLocalVelocity(0.0, 0.0, 0.15, 0.0);
            return "";
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

    Eigen::Vector3d pos, goal, little_goal, goal_diff;
    float min_height;

    std::chrono::steady_clock::time_point start_time_;
};