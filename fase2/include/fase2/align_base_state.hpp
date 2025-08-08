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


class AlignBaseState : public fsm::State {
public:
    AlignBaseState() : fsm::State(), drone(nullptr), vision(nullptr), x_pid(0,0,0,0), y_pid(0,0,0,0) {}

    void on_enter(fsm::Blackboard &blackboard){
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(!this->drone || !this->vision) return;

        this->drone->log("");
        this->drone->log("STATE: PRECISION ALIGN");

        this->align_tolerance = *blackboard.get<float>("align_tolerance");
        this->max_velocity = *blackboard.get<float>("max_horizontal_velocity");
        this->takeoff_height = *blackboard.get<float>("takeoff_height");
        this->initial_yaw = this->drone->getOrientation()[2];
        this->detection_timeout = *blackboard.get<float>("detection_timeout");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
        this->mean_base_height = *blackboard.get<float>("mean_base_height");

        this->kp = *blackboard.get<float>("pid_pos_kp");
        this->ki = *blackboard.get<float>("pid_pos_ki");
        this->kd = *blackboard.get<float>("pid_pos_kd");
        this->setpoint = *blackboard.get<float>("setpoint");

        this->package_state = *blackboard.get<std::string>("package_state");

        if (this->package_state == "get_package") {
            this->drone->log("Aligning to package base");
        } else if (this->package_state == "deliver_package") {
            this->drone->log("Aligning to delivery base");
        } else {
            this->drone->log("Unknown package state: " + this->package_state);
        }


        this->print_counter = 0;
        this->no_detection_counter = 0;
        this->approx_offset = Eigen::Vector2d::Zero();

        this->total_detected = 0;
        this->total_undetected = 0;


        this->x_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
        this->y_pid = PidController(this->kp, this->ki, this->kd, this->setpoint);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void) blackboard;
        this->print_counter++;

        this->pos = this->drone->getLocalPosition();
        this->yaw = this->drone->getOrientation()[2];
        
        if (this->print_counter % 5 == 0 ){
            this->drone->log("");
            this->drone->log("yaw=" + std::to_string(this->yaw));
            this->drone->log("Detected: " + std::to_string(this->total_detected) + ", Undetected: " + std::to_string(this->total_undetected));
        }

        if (this->vision->lastBaseDetectionTime() > this->detection_timeout){
            this->drone->log("NO DETECTION TIMEOUT EXCEEDED: " + std::to_string(this->detection_timeout) + "s.");
            return "LOST BASE";
        }

        
        if (this->vision->isThereDetection()) {
            this->total_detected++;
            
            this->no_detection_counter = 0;
            auto bbox = this->vision->getClosestBbox();
            this->approx_offset = this->getApproximateOffset(bbox);

            auto approx_base = this->pos.head<2>() + this->approx_offset;
            this->vision->publishBaseDetection("detected_base", approx_base, this->mean_base_height); 
            

            if (this->approx_offset.norm() < this->align_tolerance){
                return "PRECISELY ALIGNED";
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


        float x_rate = x_pid.compute(this->setpoint - this->approx_offset.x());
        float y_rate = y_pid.compute(this->setpoint - this->approx_offset.y());

        // Limit velocity to max_velocity
        Eigen::Vector2d rate = Eigen::Vector2d({x_rate, y_rate});
        rate = rate.norm() > this->max_velocity ? 
               rate.normalized() * this->max_velocity : rate;

        
        if (this->print_counter % 5 == 0 ){
            this->drone->log("Rates: x=" + std::to_string(rate.x()) + ", y=" + std::to_string(rate.y()));
        }

        this->drone->setLocalVelocity(rate.x(), rate.y(), 0.0, 0.0);

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
    float takeoff_height;
    float initial_yaw;
    float detection_timeout;
    float height_to_ground;
    float mean_base_height;
    
    int print_counter;
    int no_detection_counter;
    int total_detected, total_undetected;

    std::string package_state;
    
    
    Eigen::Vector3d pos;
    Eigen::Vector2d approx_offset;
    float yaw;

    Eigen::Vector2d getApproximateOffset(BoundingBox bbox) {

        double bbox_x = bbox.center_x;
        double bbox_y = bbox.center_y;

        
        // Assuming base is at mean_base_height above the ground
        // Converting to positive height value
        double height = - (this->pos.z() - this->mean_base_height);
        
        // height_to_ground: ratio between distance seen in image (from left to right) and distance from the ground (height).
        double k = std::atan(this->height_to_ground / 2);
        
        // Yolo coordinates: x -> left to right, y -> top to bottom
        double x_img = height * std::tan(k * 2 * (bbox_x - 0.5));
        double y_img = height * std::tan(k * 2 * (bbox_y - 0.5));
        
        
        // Drone coordinates: x -> front, y -> right
        double x_drone = - y_img;
        double y_drone = x_img;
        
        // FRD (Forward-Right-Down) coordinates
        double x_frd = x_drone * cos(this->yaw) - y_drone * sin(this->yaw);
        double y_frd = x_drone * sin(this->yaw) + y_drone * cos(this->yaw);

        
        if (this->print_counter % 5 == 0 ){
            this->drone->log("Bbox center: x=" + std::to_string(bbox_x) + ", y=" + std::to_string(bbox_y));
            this->drone->log("x_frd=" + std::to_string(x_frd) + ", y_frd=" + std::to_string(y_frd));
        }
        
        return Eigen::Vector2d({x_frd, y_frd});
    }
};