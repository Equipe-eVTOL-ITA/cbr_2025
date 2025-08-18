#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "PidController.hpp"
#include "vision_fase1.hpp"


class PrecisionAlignState : public fsm::State {
public:
    PrecisionAlignState() : fsm::State(), drone(nullptr), vision(nullptr), x_pid(0,0,0,0), y_pid(0,0,0,0) {}

    void on_enter(fsm::Blackboard &blackboard){
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(!this->drone || !this->vision) return;

        this->drone->log("");
        this->drone->log("STATE: PRECISION ALIGN");

        this->align_tolerance = *blackboard.get<float>("align_tolerance");
        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->align_descent_velocity = *blackboard.get<float>("align_descent_velocity");
        this->initial_yaw = this->drone->getOrientation()[2];
        this->detection_timeout = *blackboard.get<float>("detection_timeout");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
        this->mean_base_height = *blackboard.get<float>("mean_base_height");

        this->kp = *blackboard.get<float>("pid_pos_kp");
        this->ki = *blackboard.get<float>("pid_pos_ki");
        this->kd = *blackboard.get<float>("pid_pos_kd");
        this->setpoint = *blackboard.get<float>("setpoint");


        this->print_counter = 0;
        this->no_detection_counter = 0;
        this->aligned_counter = 0;
        this->horizontal_distance = 0;

        this->total_detected = 0;
        this->total_undetected = 0;

        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        this->print_counter++;

        this->pos = this->drone->getLocalPosition();
        this->orientation = this->drone->getOrientation();


        if (this->vision->lastBaseDetectionTime() > this->detection_timeout){
            this->drone->log("NO DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST BASE";
        }

        if (this->aligned_counter > 10){
            return "PRECISELY ALIGNED";
        }

        
        if (this->vision->isThereDetection()) {
            this->total_detected++;
            
            this->no_detection_counter = 0;
            auto bbox = this->vision->getClosestBbox();
            this->approx_base = this->vision->getAccurateBase(this->pos, this-> orientation, bbox);
            this->horizontal_distance = (approx_base.head<2>() - this->pos.head<2>()).norm();

            this->vision->publishBaseDetection("detected_base", approx_base); 

            if (this->horizontal_distance < this->align_tolerance){
                this->aligned_counter++;
            }
            else{
                this->aligned_counter = 0;
            }
        }
        else {
            this->total_undetected++;
            this->no_detection_counter++;
            if (this->no_detection_counter > 3) {
                this->drone->log("No detection found.");
                this->drone->setLocalPosition(this->pos.x(), this->pos.y(), this->pos.z(), this->initial_yaw);
                return "";
            }
        }

        Eigen::Vector2d diff = this->approx_base.head<2>() - this->pos.head<2>();
        float z_rate = 0.0;

        if (this->horizontal_distance < 4 * this->align_tolerance) {
            z_rate = this->align_descent_velocity;
        }
        
        float x_rate = x_pid.compute(this->setpoint - diff.x());
        float y_rate = y_pid.compute(this->setpoint - diff.y());
        
        // Limit velocity to max_velocity
        Eigen::Vector2d rate = Eigen::Vector2d({x_rate, y_rate});
        rate = rate.norm() > this->max_velocity ? 
        rate.normalized() * this->max_velocity : rate;
        
        if (this->print_counter % 5 == 0 ){
            this->drone->log("");
            this->drone->log("Detected: " + std::to_string(this->total_detected) + ", Undetected: " + std::to_string(this->total_undetected));
            this->drone->log("Rates: {" + std::to_string(rate.x()) + ", " + std::to_string(rate.y()) + ", " + std::to_string(z_rate) + "}");
        }


        this->drone->setLocalVelocity(rate.x(), rate.y(), z_rate, 0.0);


        return "";
    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    
    PidController x_pid, y_pid;
    float kp, ki, kd;
    float setpoint;
    
    float align_tolerance;
    float max_velocity;
    float align_descent_velocity;
    float initial_yaw;
    float detection_timeout;
    float height_to_ground;
    float mean_base_height;
    
    int print_counter;
    int no_detection_counter;
    int aligned_counter;
    int total_detected, total_undetected;

    float horizontal_distance;
    
    Eigen::Vector3d pos, orientation;
    Eigen::Vector3d approx_base;
};