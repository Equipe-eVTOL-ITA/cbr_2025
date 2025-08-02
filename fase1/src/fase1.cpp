#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"
#include "fase1/vision_fase1.hpp"

#include "fase1/initial_takeoff_state.hpp"
#include "fase1/search_base_state.hpp"
#include "fase1/goto_base_state.hpp"
#include "fase1/precision_align_state.hpp"
#include "fase1/landing_state.hpp"
#include "fase1/return_home_state.hpp"
#include "fase1/takeoff_state.hpp"


class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM(
        std::shared_ptr<Drone> drone,
        std::shared_ptr<VisionNode> vision,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {
        
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);

        // Parametros de ROS 2
        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }
        
        
        float takeoff_height = *this->blackboard_get<float>("takeoff_height");

        std::vector<ArenaPoint> waypoints;
        waypoints.push_back({Eigen::Vector3d({1.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -1.0, takeoff_height})});
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);
        this->blackboard_set<bool>("finished_bases", false);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
        this->add_state("GO TO BASE", std::make_unique<GoToBaseState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("PRECISION LANDING", std::make_unique<LandingState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());

        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH BASE", {
            {"BASE FOUND", "GO TO BASE"},
            {"SEARCH ENDED", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO TO BASE", {
            {"OVER THE BASE", "PRECISION ALIGN"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION ALIGN", {
            {"PRECISELY ALIGNED", "PRECISION LANDING"},
            {"LOST BASE", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION LANDING", {
            {"LANDED", "TAKEOFF"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"NEXT BASE", "SEARCH BASE"},
            {"FINISHED BASES", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("RETURN HOME", {
            {"AT HOME", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });
    }

};


class NodeFSM : public rclcpp::Node {
public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) 
        : rclcpp::Node("fase1_fsm"), drone_node_(drone), vision_node_(vision) {


        std::map<std::string, std::variant<double, std::string>> default_params = {
            {"fictual_home_x", 1.0},
            {"fictual_home_y", -0.75},
            {"fictual_home_z", 0.6},

            {"takeoff_height", -2.0},
            {"max_vertical_velocity", 1.5},
            {"max_horizontal_velocity", 1.0},
            
            {"max_search_time", 30.0},
            {"position_tolerance", 0.08},
            
            {"pid_pos_kp", 0.9},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.5},

            {"known_base_radius", 1.7},
            {"height_to_ground", 1.0},
            {"mean_base_height", 0.75},
            {"detection_timeout", 10.0},
            {"align_tolerance", 0.05},
            {"landing_timeout", 8.0},
            {"landing_velocity", 0.5}
        };
        
        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase1FSM>(drone_node_, vision_node_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this)
        );

    }

    void executeFSM() {
        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase1FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>>& defaults) {
        
        std::map<std::string, std::variant<double, std::string>> result;
        
        for (const auto& [name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }
        
        return result;
    }
};


int main(int argc, const char *argv[]){
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    
    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    auto fsm_node = std::make_shared<NodeFSM>(drone, vision);
    
    executor.add_node(drone);
    executor.add_node(vision);
    executor.add_node(fsm_node);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}