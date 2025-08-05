#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include "fsm/fsm.hpp"
#include "../fase1/Base.hpp"
#include "../fase1/ArenaPoint.hpp"
#include "drone/Drone.hpp"
#include "../fase1/vision_fase1.hpp"


class TesterState : public fsm::State {
public:
    TesterState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        this->vision = *blackboard.get<std::shared_ptr<VisionNode>>("vision");
        if(this->drone == nullptr || this->vision == nullptr) return;

        this->drone->log("");
        this->drone->log("STATE: TESTER");

        float home_x = *blackboard.get<float>("fictual_home_x");
        float home_y = *blackboard.get<float>("fictual_home_y");
        float home_z = *blackboard.get<float>("fictual_home_z");
        const Eigen::Vector3d fictual_home = Eigen::Vector3d({home_x, home_y, home_z});
        blackboard.set<Eigen::Vector3d>("home_position", fictual_home);

        this->drone->toOffboardSync();
        this->drone->setHomePosition(fictual_home);
        
        this->pos = this->drone->getLocalPosition();

        Base base{this->pos, true};
        this->vision->publishBaseDetection("confirmed_base", this->pos.head<2>(), this->pos.z());
        
        std::vector<Base> bases_vector;
        bases_vector.push_back(base);
        blackboard.set<std::vector<Base>>("bases", bases_vector);


        this->waypoints = blackboard.get<std::vector<ArenaPoint>>("waypoints");
        this->bases = blackboard.get<std::vector<Base>>("bases");

        this->print_counter = 0;

        this->known_base_radius = *blackboard.get<float>("known_base_radius");
        this->height_to_ground = *blackboard.get<float>("height_to_ground");
        this->mean_base_height = *blackboard.get<float>("mean_base_height");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        this->print_counter++;

        this->pos = this->drone->getLocalPosition();
        this->yaw = this->drone->getOrientation()[2];

        if (this->vision->isThereDetection()) {
            auto bboxes = this->vision->getDetections();

            for (const auto& bbox : bboxes) {
                const Eigen::Vector2d approx_base = getApproximateBase(bbox);
                bool is_known_base = false;
                float min_horizontal_distance = std::numeric_limits<float>::max();
                
                // Publish base detection to telemetry
                this->vision->publishBaseDetection("detected_base", approx_base, -this->mean_base_height); 
                
                if (this->print_counter % 5 == 0) {
                    this->drone->log("");
                    this->drone->log("Estimate: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                }

                for (const auto& base : *this->bases) {
                    float horizontal_distance = (approx_base - base.coordinates.head<2>()).norm();

                    min_horizontal_distance = std::min(min_horizontal_distance, horizontal_distance);
                    
                    if (this->print_counter % 5 == 0) {
                    }
                    
                    if (horizontal_distance < this->known_base_radius) {
                        if (this->print_counter % 5 == 0) {
                            std::string base_index = std::to_string(&base - &(*this->bases)[0]);
                            this->drone->log("Dist " + std::to_string(horizontal_distance) + " to Base {" + base_index + "}: {"
                                        + std::to_string(base.coordinates[0]) + ", " + std::to_string(base.coordinates[1]) + "}");
                            this->drone->log("Known base!");
                        }
                        is_known_base = true;
                        break;
                    }
                }

                if (!is_known_base) {
                    this->drone->log("");
                    this->drone->log("New base at: {" + std::to_string(approx_base.x()) + ", " + std::to_string(approx_base.y()) + "}");
                    this->drone->log("Min distance to known base: " + std::to_string(min_horizontal_distance));
                    
                    // Publish first detection as yconfirmed marker for now
                    this->vision->publishBaseDetection("confirmed_base", approx_base, -this->mean_base_height); 

                    bases->push_back({Eigen::Vector3d({approx_base.x(), approx_base.y(), -this->mean_base_height}), true});
                    blackboard.set<std::vector<Base>>("bases", *this->bases);
                    
                    return "";
                }
            }
        } 

        return "";

    }

private:
    std::shared_ptr<Drone> drone;
    std::shared_ptr<VisionNode> vision;
    std::vector<ArenaPoint>* waypoints;
    std::vector<Base>* bases;

    float yaw;

    int print_counter;

    float known_base_radius;
    float height_to_ground;
    float mean_base_height;

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

        // Assuming base is at mean_base_height (positive value) above the ground
        double height = -this->pos.z() - this->mean_base_height;

        // height_to_ground: ratio between distance seen in image (from left to right) and distance from the ground (height).
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