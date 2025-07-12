#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "PidController.hpp"

class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr), vision(nullptr), x_pid(0,0,0,0), y_pid(0,0,0,0) {}

    void on_enter(fsm::Blackboard &blackboard){
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(!this->drone || !this->vision) return;
        this->drone->log("STATE: Alinhando com a base no chao...");

        this->position_tolerance = *blackboard.get<float>("position_tolerance");
        this->kp = *blackboard.get<float>("pid_pos_kp");
        this->ki = *blackboard.get<float>("pid_pos_ki");
        this->kd = *blackboard.get<float>("pid_pos_kd");
        this->setpoint = *blackboard.get<float>("setpoint");

        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        std::string class_id = *blackboard.get<std::string>("class_id");

        float x_rate = 0.0, y_rate = 0.0;
        float vertical_distance = 1.0;

        float yaw = drone->getOrientation()[2];

        // Usar VisionNode em vez da classe Detection
        auto detection_result = vision->getClosestDetection("vertical", class_id);

        if (detection_result.has_detection){
            BoundingBox vertical_bbox = detection_result.closest_bbox;
            vertical_distance = detection_result.min_distance;

            x_rate = x_pid.compute(-vertical_bbox.center_y);
            y_rate = y_pid.compute(vertical_bbox.center_x);
        }
        if (vertical_distance < this->position_tolerance){
            return "OVER THE BASE";
        }

        float frd_x_rate = x_rate * cos(yaw) - y_rate * sin(yaw);
        float frd_y_rate = x_rate * sin(yaw) + y_rate * cos(yaw);
        drone->setLocalVelocity(frd_x_rate, frd_y_rate, 0, 0);

        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    PidController x_pid, y_pid;
    float kp, ki, kd;
    float setpoint;
    float position_tolerance;
};