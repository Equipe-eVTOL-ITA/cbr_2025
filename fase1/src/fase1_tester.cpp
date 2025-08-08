#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"

#include "fase1/vision_fase1.hpp"
#include "test/tester_state.hpp"


class Fase1TesterFSM : public fsm::FSM {
public:
    Fase1TesterFSM(
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
        
        // DYNAMIC GRID ----------------------------------------------------

        float takeoff_height = *this->blackboard_get<float>("takeoff_height");
        float home_x = *this->blackboard_get<float>("fictual_home_x");
        float home_y = *this->blackboard_get<float>("fictual_home_y");
        float y_length = *this->blackboard_get<float>("grid_y_length");
        float step_x = *this->blackboard_get<float>("grid_step_x");
        float num_steps = *this->blackboard_get<float>("grid_num_steps");

        int state = 0;
        int num_steps_taken = 0;
        bool finished = false;
        float x_coord = home_x;
        float y_coord = home_y;

        std::vector<ArenaPoint> waypoints;
        while (num_steps_taken < num_steps || !finished) {
            if (state % 4 == 0) {
                // Go left
                y_coord = home_y + y_length;
                finished = true;
            } else if (state % 4 == 1 || state % 4 == 3) {
                // Go front
                x_coord += step_x;
                num_steps_taken++;
                finished = false;
            } else {
                // Go right
                y_coord = home_y;
                finished = true;
            }
            state++;
            waypoints.push_back({Eigen::Vector3d({x_coord, y_coord, takeoff_height})});
        }
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);
        this->blackboard_set<bool>("finished_bases", false);

        // ------------------------------------------------------------------


        // STATE MACHINE ----------------------------------------------------


        this->add_state("TESTER", std::make_unique<TesterState>());

        this->add_transitions("TESTER", {
            {"TEST COMPLETE", "FINISHED"},
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
            {"fictual_home_y", -1.0},
            {"fictual_home_z", -0.6},

            {"num_bases", 6.0},
            {"known_base_radius", 1.5},
            {"height_to_ground", 1.2},
            
            {"grid_y_length", -6.0},
            {"grid_step_x", 2.0},
            {"grid_num_steps", 3.0},

            {"takeoff_height", -2.5},
            {"align_height", -2.0},
            {"max_base_height", -1.5},
            {"mean_base_height", -0.85},

            {"max_vertical_velocity", 1.2},
            {"max_horizontal_velocity", 1.0},
            {"landing_velocity_max", 0.5},
            {"landing_velocity_min", 0.2},

            {"max_search_time", 30.0},
            {"detection_timeout", 10.0},

            {"position_tolerance", 0.07},
            {"align_tolerance", 0.025},

            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.01},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.0}
        };
        
        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase1TesterFSM>(drone_node_, vision_node_, params);

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
    std::unique_ptr<Fase1TesterFSM> fsm_;
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