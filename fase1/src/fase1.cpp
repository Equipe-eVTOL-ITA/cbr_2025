#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"
#include "vision_fase1.cpp"

#include "fase1/initial_takeoff_state.hpp"
#include "fase1/search_base_state.hpp"
#include "fase1/goto_base_state.hpp"
#include "fase1/precision_align_state.hpp"
#include "fase1/landing_state.hpp"

class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM(
        std::shared_ptr<Drone> drone,
        std::shared_ptr<VisionNode> vision,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {
        
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);

        const Eigen::Vector3d orientation = drone->getOrientation();
        
        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }
        
        // Parâmetros especiais (não vindos do mapa)
        std::vector<Base> bases;
        bases.push_back({drone->getLocalPosition(), true});
        
        this->blackboard_set<float>("initial_yaw", orientation[2]);
        this->blackboard_set<std::vector<Base>>("bases", bases);
        this->blackboard_set<bool>("finished_bases", false);

        // Construir waypoints usando parâmetros já no blackboard
        float square_side_length = std::get<double>(params.at("square_side_length"));
        float takeoff_height = std::get<double>(params.at("takeoff_height"));
        
        // Pontos da Arena
        /*
        E---A---B
        |       |
        |       |
        D-------C
        */
        std::vector<ArenaPoint> waypoints;
        float l2 = square_side_length/2.0;
        waypoints.push_back({
            Eigen::Vector3d({l2, 0, takeoff_height}) // A
        });
        waypoints.push_back({
            Eigen::Vector3d({l2, l2, takeoff_height}) // B
        });
        waypoints.push_back({
            Eigen::Vector3d({-l2, l2, takeoff_height}) // C
        });
        waypoints.push_back({
            Eigen::Vector3d({-l2, -l2, takeoff_height}) // D
        });
        waypoints.push_back({
            Eigen::Vector3d({l2, -l2, takeoff_height}) // E
        });
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASE", std::make_unique<SearchBaseState>());
        this->add_state("GO TO BASE", std::make_unique<GoToBaseState>());
        this->add_state("PRECISION ALIGN", std::make_unique<PrecisionAlignState>());
        this->add_state("PRECISION LANDING", std::make_unique<LandingState>());

        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "SEARCH BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH BASE", {
            {"BASE FOUND", "GO TO BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO TO BASE", {
            {"OVER THE BASE", "PRECISION ALIGN"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION ALIGN", {
            {"PRECISELY ALIGNED", "PRECISION LANDING"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PRECISION LANDING", {
            {"LANDED", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });
    }

};


class NodeFSM : public rclcpp::Node {
public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) 
        : rclcpp::Node("fase1_fsm"), drone_node_(drone), vision_node_(vision) {


        std::map<std::string, std::variant<double, std::string>> default_params = {
            {"takeoff_height", -2.0},
            {"max_vertical_velocity", 1.5},
            {"max_horizontal_velocity", 1.0},
            {"resolution_x", 800.0},
            {"resolution_y", 800.0},
            {"square_side_length", 2.0},
            
            {"max_search_time", 30.0},
            {"class_id", std::string("estrela")},
            {"position_tolerance", 0.08},
            
            {"pid_pos_kp", 0.9},
            {"pid_pos_ki", 0.0},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.5}
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