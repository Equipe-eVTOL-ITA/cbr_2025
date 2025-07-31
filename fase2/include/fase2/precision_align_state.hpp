#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "PidController.hpp"
#include "vision_fase2.hpp"


class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr), vision(nullptr), x_pid(0,0,0,0), y_pid(0,0,0,0) {}

    void on_enter(fsm::Blackboard &blackboard){
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(!this->drone || !this->vision) return;
        this->drone->log("STATE: PRECISION ALIGN");

        this->align_tolerance = *blackboard.get<float>("align_tolerance");
        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->takeoff_height = *blackboard.get<float>("takeoff_height");
        this->initial_yaw = this->drone->getOrientation()[2];
        this->detection_timeout = *blackboard.get<float>("detection_timeout");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");

        this->kp = *blackboard.get<float>("pid_pos_kp");
        this->ki = *blackboard.get<float>("pid_pos_ki");
        this->kd = *blackboard.get<float>("pid_pos_kd");
        this->setpoint = *blackboard.get<float>("setpoint");

        this->x_pid = PidController(this->kp, this->ki, this->kd, 0.0);
        this->y_pid = PidController(this->kp, this->ki, this->kd, 0.0);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        float x_rate = 0.0, y_rate = 0.0;

        this->pos = this->drone->getLocalPosition();
        this->yaw = this->drone->getOrientation()[2];

        if (this->vision->lastBaseDetectionTime() > this->detection_timeout){
            this->drone->log("NO DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST BASE";
        }


        if (!this->vision->isThereDetection()){
            this->drone->log("No detection found.");
            this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->initial_yaw);
            return "";
        }

        auto bbox = vision->getClosestBbox();
        auto approx_offset = getApproximateOffset(bbox);

        if (approx_offset.norm() < this->align_tolerance){
            return "PRECISELY ALIGNED";
        }

        x_rate = x_pid.compute(approx_offset.x());
        y_rate = y_pid.compute(approx_offset.y());

        this->drone->setLocalVelocity(x_rate, y_rate, 0.0, 0.0);

        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    PidController x_pid, y_pid;

    float align_tolerance;
    float max_velocity;
    float takeoff_height;
    float initial_yaw;
    float detection_timeout;
    float height_to_ground;

    float kp, ki, kd;
    float setpoint;

    Eigen::Vector3d pos;
    float yaw;

    Eigen::Vector2d getApproximateOffset(BoundingBox bbox) {

        double bbox_x = bbox.center_x;
        double bbox_y = bbox.center_y;

        // Assuming base is at 0.75m above the ground
        double height = -this->pos.z() - 0.75;

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
        
        return Eigen::Vector2d({frd_x, frd_y});
    }
};