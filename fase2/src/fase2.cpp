#include <memory>
#include <iostream>
#include <vector>

#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"

#include "fase2/align_base_state.hpp"
#include "fase2/align_package_state.hpp"
#include "fase2/approach_package_state.hpp"
#include "fase2/ArenaPoint.hpp"
#include "fase2/Base.hpp"
#include "fase2/garra_state.hpp"
#include "fase2/goto_delivery_state.hpp"
#include "fase2/goto_package_state.hpp"
#include "fase2/initial_takeoff_state.hpp"
#include "fase2/landing_state.hpp"
#include "fase2/lost_state.hpp"
#include "fase2/return_home_state.hpp"
#include "fase2/takeoff_state.hpp"
#include "fase2/vision_fase2.hpp"


class Fase2FSM : public fsm::FSM {
public:
    Fase2FSM(
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
        

        // PACKAGE AND DELIVERY WAYPOINTS-----------------------------------

        std::vector<ArenaPoint> packages;
        packages.emplace_back(
            *this->blackboard_get<float>("package_1_x"),
            *this->blackboard_get<float>("package_1_y"),
            *this->blackboard_get<float>("package_height")
        );
        packages.emplace_back(
            *this->blackboard_get<float>("package_2_x"),
            *this->blackboard_get<float>("package_2_y"),
            *this->blackboard_get<float>("package_height")
        );
        packages.emplace_back(
            *this->blackboard_get<float>("package_3_x"),
            *this->blackboard_get<float>("package_3_y"),
            *this->blackboard_get<float>("package_height")
        );

        std::vector<ArenaPoint> deliveries;
        deliveries.emplace_back(
            *this->blackboard_get<float>("delivery_1_x"),
            *this->blackboard_get<float>("delivery_1_y"),
            *this->blackboard_get<float>("delivery_height")
        );
        deliveries.emplace_back(
            *this->blackboard_get<float>("delivery_2_x"),
            *this->blackboard_get<float>("delivery_2_y"),
            *this->blackboard_get<float>("delivery_height")
        );
        deliveries.emplace_back(
            *this->blackboard_get<float>("delivery_3_x"),
            *this->blackboard_get<float>("delivery_3_y"),
            *this->blackboard_get<float>("delivery_height")
        );

        this->blackboard_set<std::vector<ArenaPoint>>("packages", packages);
        this->blackboard_set<std::vector<ArenaPoint>>("deliveries", deliveries);

        // ------------------------------------------------------------------


        // STATE MACHINE ----------------------------------------------------

        // GET PACKAGE--------

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("GO TO PACKAGE", std::make_unique<GoToPackageState>());
        this->add_state("ALIGN BASE", std::make_unique<AlignBaseState>());
        this->add_state("APPROACH PACKAGE", std::make_unique<ApproachPackageState>());
        this->add_state("ALIGN PACKAGE", std::make_unique<AlignBaseState>());
        this->add_state("GARRA", std::make_unique<GarraState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());

        // DELIVER PACKAGE------

        this->add_state("GO TO DELIVERY", std::make_unique<GoToDeliveryState>());
        // ALIGN BASE
        this->add_state("LANDING", std::make_unique<LandingState>());
        // GARRA
        // TAKEOFF

        // FINISH--------------
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // FALLBACK IF DETECTIONS ARE LOST
        this->add_state("LOST", std::make_unique<LostState>());


        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "GO TO PACKAGE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO TO PACKAGE", {
            {"OVER THE BASE", "ALIGN BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN BASE", {
            // GETTING PACKAGE
            {"APPROACH PACKAGE", "APPROACH PACKAGE"},
            {"LOST", "LOST"},
            // DELIVERING PACKAGE
            {"LAND", "LANDING"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("APPROACH PACKAGE", {
            {"APPROACH COMPLETE", "ALIGN PACKAGE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN PACKAGE", {
            {"AT PACKAGE", "GARRA"},
            {"LOST PACKAGE", "LOST"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GARRA", {
            {"GARRA MOVEMENT COMPLETE", "TAKEOFF"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"DELIVER PACKAGE", "GO TO DELIVERY"},
            {"GET NEXT PACKAGE", "GO TO PACKAGE"},
            {"FINISHED PACKAGES", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GO TO DELIVERY", {
            {"AT DELIVERY BASE", "ALIGN BASE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("RETURN HOME", {
            {"AT HOME", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LOST", {
            {"FOUND PACKAGE BASE", "ALIGN BASE"},
            {"FOUND DELIVERY BASE", "ALIGN BASE"}, 
            {"FOUND PACKAGE", "ALIGN PACKAGE"}
        });

    }

};


class NodeFSM : public rclcpp::Node {
public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) 
        : rclcpp::Node("fase2_fsm"), drone_node_(drone), vision_node_(vision) {


        std::map<std::string, std::variant<double, std::string>> default_params = {
            {"fictual_home_x", 1.0},
            {"fictual_home_y", -1.0},
            {"fictual_home_z", -0.6},

            {"package_1_x", 3.0},
            {"package_1_y", -1.0},
            {"package_2_x", 5.0},
            {"package_2_y", -1.0},
            {"package_3_x", 7.0},
            {"package_3_y", -1.0},

            {"delivery_1_x", 1.0},
            {"delivery_1_y", -4.0},
            {"delivery_2_x", 4.25},
            {"delivery_2_y", -6.0},
            {"delivery_3_x", 6.5},
            {"delivery_3_y", -7.0},

            {"takeoff_height", -3.0},
            {"align_package_height", -2.0},
            {"package_height", -1.5},
            {"delivery_height", 0.0},

            {"max_vertical_velocity", 1.0},
            {"max_horizontal_velocity", 1.0},
            {"descent_velocity", 0.15},
            {"landing_velocity", 0.5},

            {"garra_timeout", 5.0},
            {"detection_timeout", 10.0},
            {"landing_timeout", 7.0},

            {"position_tolerance", 0.07},
            {"align_base_tolerance", 0.05},
            {"align_package_tolerance", 0.02},

            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.01},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.0},

            {"known_base_radius", 1.5},
            {"height_to_ground", 1.2}

        };
        
        auto params = declareAndGetParameters(default_params);

        fsm_ = std::make_unique<Fase2FSM>(drone_node_, vision_node_, params);

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
    std::unique_ptr<Fase2FSM> fsm_;
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