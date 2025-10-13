#include <memory>
#include <iostream>
#include <vector>

// bibliotecas ROS
#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"

// Auxiliares
#include "fase4/aux/vision_fase4.hpp"
#include "fase4/aux/ArenaPoint.hpp"

// Estados
#include "fase4/states/arming_state.hpp"
#include "fase4/states/takeoff_state.hpp"
#include "fase4/states/enter_into_house.hpp"
#include "fase4/states/align_window_state.hpp"
#include "fase4/states/through_window_state.hpp"
#include "fase4/states/return_home_state.hpp"
#include "fase4/states/landing_state.hpp"
#include "fase4/states/search_qr_code_state.hpp"
#include "fase4/states/bouncing_search_state.hpp"
#include "fase4/states/look_to_window_state.hpp"

typedef std::map<std::string, std::variant<double, std::string, bool>> BlackboardMap;

template<typename DoubleHandler, typename StringHandler, typename BooleanHandler>
BlackboardMap actOnBlackboardMap(
    const BlackboardMap& defaults,
    DoubleHandler&& double_handler,
    StringHandler&& string_handler,
    BooleanHandler&& boolean_handler) {

    BlackboardMap result;

    for (const auto& [name, default_value] : defaults) {
        if (std::holds_alternative<double>(default_value)) {
            result[name] = double_handler(name, std::get<double>(default_value));
        } else if (std::holds_alternative<std::string>(default_value)) {
            result[name] = string_handler(name, std::get<std::string>(default_value));
        } else if (std::holds_alternative<bool>(default_value)) {
            result[name] = boolean_handler(name, std::get<bool>(default_value));
        }
    }
    
    return result;
}

class Fase4FSM : public fsm::FSM {
private:
    std::vector<ArenaPoint> waypoints;

public:
    Fase4FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision, const BlackboardMap& parameters) : fsm::FSM({"ERROR", "FINISHED"}){
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<std::shared_ptr<VisionNode>>("vision", vision);
        
        // Armazenar parâmetros diretamente no blackboard (já processados do YAML)
        actOnBlackboardMap(
            parameters,
            // Lambda para converter double para float e armazenar no blackboard
            [this](const std::string& name, double value) -> double {
                this->blackboard_set<float>(name, static_cast<float>(value));
                return value;
            },
            // Lambda para armazenar strings no blackboard
            [this](const std::string& name, const std::string& value) -> std::string {
                this->blackboard_set<std::string>(name, value);
                return value;
            },
            [this](const std::string& name, bool value) -> bool {
                this->blackboard_set<bool>(name, value);
                return value;
            }
        );

        std::shared_ptr<std::vector<ArenaPoint>> checkpoints = std::make_shared<std::vector<ArenaPoint>>(this->config_checkpoints(12));
        this->blackboard_set<std::shared_ptr<std::vector<ArenaPoint>>>("checkpoints", checkpoints);

        // Inicializar window_position com valor padrão
        Eigen::Vector3d default_window_position(0.0, 0.0, 0.0);
        this->blackboard_set<Eigen::Vector3d>("window_position", default_window_position);

        ArenaPoint front_of_the_window = this->setFrontOfTheWindow();
        this->blackboard_set<ArenaPoint>("front_of_the_window", front_of_the_window);
        

        // Máquina de Estados
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("ENTER HOUSE", std::make_unique<EnterIntoHouseState>());
        this->add_state("ALIGN WITH WINDOW", std::make_unique<AlignWindowState>());
        this->add_state("THROUGH WINDOW", std::make_unique<ThroughWindowState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());
        this->add_state("LAND", std::make_unique<LandingState>());
        this->add_state("SEARCH QR CODE", std::make_unique<SearchQRCodeState>());
        this->add_state("BOUNCING SEARCH", std::make_unique<BouncingSearchState>());
        this->add_state("LOOK TO WINDOW", std::make_unique<LookToWindowState>());

        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKENOFF", "LAND"},
            {"ENTER HOUSE", "ENTER HOUSE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ENTER HOUSE", {
            {"ALIGN WITH WINDOW", "ALIGN WITH WINDOW"}
        });

        this->add_transitions("LOOK TO WINDOW", {
            {"LOOKING", "ALIGN WITH WINDOW"},
            {"SEG FAULT", "ERROR"}
        });
        
        this->add_transitions("ALIGN WITH WINDOW", {
            {"ALIGNED", "THROUGH WINDOW"},
            {"NO WINDOW DETECTED", "LAND"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("THROUGH WINDOW", {
            {"PASSED", "SEARCH QR CODE"},
            {"NOT PASSED", "LAND"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH QR CODE", {
            {"BOUNCING SEARCH", "BOUNCING SEARCH"},
            {"RETURN HOME", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("BOUNCING SEARCH", {
            {"BOUNCING SEARCH COMPLETED", "SEARCH QR CODE"},
            {"LOOK TO WINDOW", "LOOK TO WINDOW"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("RETURN HOME", {
            {"ARRIVED", "LAND"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LAND", {
            {"LANDED", "FINISHED"},
            {"NOT LANDED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });
    }

private:

    ArenaPoint setFrontOfTheWindow() {
        ArenaPoint point{};
        
        float x = *this->blackboard_get<float>("front_of_the_window_x");
        float y = *this->blackboard_get<float>("front_of_the_window_y");
        float z = *this->blackboard_get<float>("mean_height");
        
        point = ArenaPoint(x, y, z);

        return point;
    }


    std::vector<ArenaPoint> config_checkpoints(int number_of_checkpoints){
        float h = *this->blackboard_get<float>("mean_height");
        float x, y;

        std::vector<ArenaPoint> points;
        std::vector<bool> esperam_janelas;

        for(int i = 1; i <= number_of_checkpoints; i++){
            ArenaPoint point{}; // evitar vexing parse

            // eg: checkpoint_1_x, checkpoint_2_x, etc.
            std::string wp_num = "checkpoint_" + std::to_string(i) + "_";
            x = *this->blackboard_get<float>(wp_num + "x");
            y = *this->blackboard_get<float>(wp_num + "y");

            point = ArenaPoint(x, y, h);

            points.push_back(point);

            esperam_janelas.push_back(*this->blackboard_get<bool>("checkpoint_"+std::to_string(i)+"_espera_janela"));
        }

        std::shared_ptr<std::vector<bool>> checkpoints_espera_janela = std::make_shared<std::vector<bool>>(esperam_janelas);
        this->blackboard_set<std::shared_ptr<std::vector<bool>>>("checkpoints_espera_janela", checkpoints_espera_janela);
        
        return points;

    }

};

class NodeFSM : public rclcpp::Node {
private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase4FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) : rclcpp::Node("fase4_fsm"), drone_node_(drone), vision_node_(vision) {
        
        BlackboardMap defaults = {
            // === Posições Home e Waypoints ===
            {"fictual_home_x", 1.0},
            {"fictual_home_y", -1.0},
            {"fictual_home_z", -0.6},
            
            // Return Home
            {"at_home", false},
            {"is_returning_home", false},

            // front of the first window
            {"front_of_the_window_x", 3.0},
            {"front_of_the_window_y", 0.0},

            // === 13 Checkpoints ===
            {"checkpoint_1_x", 3.0},
            {"checkpoint_1_y", -3.0},
            {"checkpoint_1_espera_janela", true},

            {"checkpoint_2_x", 5.0},
            {"checkpoint_2_y", -5.0},
            {"checkpoint_2_espera_janela", true},

            {"checkpoint_3_x", 7.0},
            {"checkpoint_3_y", -7.0},
            {"checkpoint_3_espera_janela", true},

            {"checkpoint_4_x", 9.0},
            {"checkpoint_4_y", -9.0},
            {"checkpoint_4_espera_janela", true},

            {"checkpoint_5_x", 11.0},
            {"checkpoint_5_y", -11.0},
            {"checkpoint_5_espera_janela", true},

            {"checkpoint_6_x", 13.0},
            {"checkpoint_6_y", -13.0},
            {"checkpoint_6_espera_janela", true},

            {"checkpoint_7_x", 15.0},
            {"checkpoint_7_y", -15.0},
            {"checkpoint_7_espera_janela", true},

            {"checkpoint_8_x", 17.0},
            {"checkpoint_8_y", -17.0},
            {"checkpoint_8_espera_janela", true},

            {"checkpoint_9_x", 19.0},
            {"checkpoint_9_y", -19.0},
            {"checkpoint_9_espera_janela", true},

            {"checkpoint_10_x", 21.0},
            {"checkpoint_10_y", -21.0},
            {"checkpoint_10_espera_janela", true},

            {"checkpoint_11_x", 23.0},
            {"checkpoint_11_y", -23.0},
            {"checkpoint_11_espera_janela", true},

            {"checkpoint_12_x", 25.0},
            {"checkpoint_12_y", -25.0},
            {"checkpoint_12_espera_janela", true},
            // index
            {"checkpoint_index", 0.0},
            {"espera_janela", false},

            // === Alturas e Velocidades ===
            {"mean_height", -1.0},
            {"takeoff_height", -2.5},
            {"max_vertical_velocity", 1.0},
            {"max_horizontal_velocity", 1.0},
            {"landing_velocity", 0.5},
            {"search_velocity", 0.5},
            {"max_yaw_rate", 0.5},

            // === Tolerâncias ===
            {"position_tolerance", 0.07},
            {"takeoff_tolerance", 0.3},        // Tolerância específica para takeoff (30cm)
            {"align_tolerance", 0.15},         // Tolerância para alinhamento com objetos (15cm)

            // === Timeouts ===
            {"detection_timeout", 10.0},
            {"landing_timeout", 7.0},

            // === Controladores PID - Posição ===
            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.01},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.0},

            // === Controladores PID - Yaw ===
            {"pid_yaw_kp", 2.0},
            {"pid_yaw_ki", 0.0},
            {"pid_yaw_kd", 0.1},

            // === Parâmetros de Visão ===
            {"height_to_ground", 1.2},
            {"mean_base_height", 0.0},
            {"known_base_radius", 1.5},

            // === Estados e Flags ===
            {"initial_takeoff_taken", false},   // Flag se já decolou
            {"current_checkpoint_index", 0.0},  // Índice do checkpoint atual

            // pass through window
            {"pass_through_speed", 0.3},       // Velocidade ao passar pela janela
            {"pass_through_distance", 1.5},    // Distância a percorrer ao passar pela janela
            {"pass_through_tolerance", 0.2},   // Tolerância para considerar que passou pela janela
            {"pass_through_timeout", 10.0},    // Timeout para passar pela janela

            // window alignment offsets
            {"window_align_offset_x", 0.0},
            {"window_align_offset_y", 0.0},

            // state machine control
            {"last_state", std::string("")},   // Para return_to_last_state_state
            
            // === Outras Configurações ===
            {"descent_velocity", 0.15},          // Velocidade de descida
            {"align_descent_velocity", 0.12},    // Velocidade de descida durante alinhamento

            // === Parâmetros Bouncing Search ===
            {"up_height", -0.5},                 // Altura para fases "UP" (metros, negativo para cima no NED)
            {"down_height", -1.5},               // Altura para fases "DOWN" (metros, negativo para baixo no NED)
            {"middle_height", -1.0},             // Altura para fases "MIDDLE" (metros, intermediária)

            // === Parâmetros das Janelas ===
            {"janela_counter", 0.0},
            {"janela_A", true},
            {"janela_B", false},
            {"janela_C", false},
            {"janela_D", true},
            {"janela_E", false},
            {"janela_F", true},
            {"janela_G", false},
            {"janela_H", true},
            {"janela_alta_height", 1.0},
            {"janela_baixa_height", 0.5}
        };

        // Cria e configura a FSM com os parâmetros
        auto parameters = actOnBlackboardMap(
            defaults,
            [this](const std::string& name, double default_value) -> double {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_double();
            },
            [this](const std::string& name, const std::string& default_value) -> std::string {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_string();
            },
            [this](const std::string& name, bool default_value) -> bool {
                this->declare_parameter(name, default_value);
                return this->get_parameter(name).as_bool();
            }
        );

        fsm_ = std::make_unique<Fase4FSM>(drone_node_, vision_node_, parameters);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this)
        );
    }

    void executeFSM(){
        if(rclcpp::ok() && !fsm_->is_finished())
            fsm_->execute();
        else
            rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);    
    rclcpp::executors::MultiThreadedExecutor executor;

    auto drone = std::make_shared<Drone>();
    auto vision = std::make_shared<VisionNode>();
    auto fsm_node = std::make_shared<NodeFSM>(drone, vision);

    executor.add_node(fsm_node);
    executor.add_node(drone);
    executor.add_node(vision);

    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}
