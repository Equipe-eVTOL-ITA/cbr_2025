#include <memory>
#include <iostream>
#include <vector>

// bibliotecas ROS2
#include "fsm/fsm.hpp"
#include <rclcpp/rclcpp.hpp>
#include "drone/Drone.hpp"

// Auxiliares
#include "fase2/aux/vision_fase2.hpp"
#include "fase2/aux/ArenaPoint.hpp"
#include "fase2/aux/PairOfArenaPoints.hpp"

// Estados
#include "fase2/states/arming_state.hpp"
#include "fase2/states/takeoff_state.hpp"
#include "fase2/states/next_base_state.hpp"
#include "fase2/states/next_deliver_state.hpp"
#include "fase2/states/align_base_state.hpp"
#include "fase2/states/align_package_state.hpp"
#include "fase2/states/landing_state.hpp"
#include "fase2/states/package_state.hpp"
#include "fase2/states/spiral_search_state.hpp"

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

class Fase2FSM : public fsm::FSM {
private:
    std::vector<ArenaPoint> waypoints;

public:
    Fase2FSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision, const BlackboardMap& parameters) : fsm::FSM({"ERROR", "FINISHED"}){
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

        // Configura os pares de waypoints (base e deliver)
        std::shared_ptr<std::vector<PairOfArenaPoints>> pairs = std::make_shared<std::vector<PairOfArenaPoints>>(this->config_pair_of_waypoints('A', 'C')); // 3 pacotes
        this->blackboard_set<std::shared_ptr<std::vector<PairOfArenaPoints>>>("waypoint_pairs", pairs);

        // lista de bases que já foram pousadas
        std::shared_ptr<std::vector<Base>> bases_pousadas = std::make_shared<std::vector<Base>>();
        this->blackboard_set<std::shared_ptr<std::vector<Base>>>("bases_pousadas", bases_pousadas);


        // configura os waypoints para busca em espiral
        // na implementacao, somar à posicao do drone
        float search_radius = *this->blackboard_get<float>("search_radius");
        float angle_increment = *this->blackboard_get<float>("angle_increment");
        float max_radius = *this->blackboard_get<float>("max_radius");

        std::shared_ptr<std::vector<ArenaPoint>> spiral_waypoints = std::make_shared<std::vector<ArenaPoint>>(generate_spiral_waypoints(angle_increment, search_radius, max_radius));
        this->blackboard_set<std::shared_ptr<std::vector<ArenaPoint>>>("spiral_waypoints", spiral_waypoints);


        // Máquina de Estados
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("NEXT BASE", std::make_unique<NextBaseState>());
        this->add_state("NEXT DELIVER", std::make_unique<NextDeliverState>());
        this->add_state("ALIGN TO BASE", std::make_unique<AlignBaseState>());
        this->add_state("ALIGN TO PACKAGE", std::make_unique<AlignPackageState>());
        this->add_state("LAND", std::make_unique<LandingState>());
        this->add_state("PACKAGE", std::make_unique<PackageState>());
        this->add_state("SPIRAL SEARCH", std::make_unique<SpiralSearchState>());

        this->set_initial_state("ARMING");

        // Transições de Estados
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"NOT ARMED", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"NEXT BASE", "NEXT BASE"},
            {"DELIVER PACKAGE", "NEXT DELIVER"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LAND", {
            {"LANDED", "PACKAGE"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("NEXT BASE", {
            {"ALIGN TO PACKAGE", "ALIGN TO PACKAGE"},
            {"NOT FOUND", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("NEXT DELIVER", {
            {"ALIGN TO DELIVER", "ALIGN TO BASE"},
            {"NOT FOUND", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SPIRAL SEARCH", {
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN TO PACKAGE", {
            {"ALIGNED", "LAND"},
            {"LOST PACKAGE", "ERROR"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("ALIGN TO BASE", {
            {"ALIGNED", "LAND"},
            {"LOST BASE", "FINISHED"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("PACKAGE", {
            {"DONE", "TAKEOFF"},
            {"SEG FAULT", "ERROR"}
        });

    }

private:

    std::vector<PairOfArenaPoints> config_pair_of_waypoints(char alpha, char omega){
        // usar somente chars maiusculos

        float h = *this->blackboard_get<float>("takeoff_height");
        float x, y;

        std::vector<PairOfArenaPoints> pairs;

        for(char c = alpha; c<=omega; c++){
            // Fix: Use uniform initialization to avoid most vexing parse
            PairOfArenaPoints pair{ArenaPoint{}, ArenaPoint{}};

            std::string char_str(1, c);  // Fix: create string from char
            
            // eg: base_A_x
            std::string wp_char = "base_"+char_str+"_";
            x = *this->blackboard_get<float>(wp_char+"x");
            y = *this->blackboard_get<float>(wp_char+"y");
            // Fix: Use proper ArenaPoint constructor (assuming it takes x, y, z)
            pair.base = ArenaPoint(x, y, h);

            // eg: deliver_A_x
            wp_char = "deliver_"+char_str+"_";
            x = *this->blackboard_get<float>(wp_char+"x");
            y = *this->blackboard_get<float>(wp_char+"y");
            // Fix: Use proper ArenaPoint constructor
            pair.deliver = ArenaPoint(x, y, h);

            pairs.push_back(pair);
        }

        return pairs;
    }

    std::vector<ArenaPoint> generate_spiral_waypoints(float deltaTheta, float deltaRadius, float maxRadius){
        std::vector<ArenaPoint> waypoints;

        float currentRadius = 0.0f;
        float currentAngle = 0.0f;

        while (currentRadius <= maxRadius) {
            float x = currentRadius * cos(currentAngle);
            float y = currentRadius * sin(currentAngle);
            // Fix: Create ArenaPoint with proper constructor
            waypoints.push_back(ArenaPoint(x, y, 0.0f)); 

            currentAngle += deltaTheta;
            currentRadius += deltaRadius;
        }

        return waypoints;
    }


};

class NodeFSM : public rclcpp::Node {
private:
    std::shared_ptr<Drone> drone_node_;
    std::shared_ptr<VisionNode> vision_node_;
    std::unique_ptr<Fase2FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    NodeFSM(std::shared_ptr<Drone> drone, std::shared_ptr<VisionNode> vision) : rclcpp::Node("fase2_fsm"), drone_node_(drone), vision_node_(vision) {
        
        BlackboardMap defaults = {
            // === Posições Home e Waypoints ===
            {"fictual_home_x", 1.0},
            {"fictual_home_y", -1.0},
            {"fictual_home_z", -0.6},
            
            // Coordenadas das bases (A, B, C)
            {"base_A_x", 3.0},
            {"base_A_y", -1.0},
            {"base_B_x", 5.0},
            {"base_B_y", -1.0},
            {"base_C_x", 7.0},
            {"base_C_y", -1.0},
            
            // Coordenadas das entregas (A, B, C)
            {"deliver_A_x", 1.0},
            {"deliver_A_y", -4.0},
            {"deliver_B_x", 4.25},
            {"deliver_B_y", -6.0},
            {"deliver_C_x", 6.5},
            {"deliver_C_y", -7.0},

            // === Alturas e Velocidades ===
            {"takeoff_height", -3.0},
            {"align_package_height", -2.0},
            {"package_height", -1.5},
            {"delivery_height", 0.0},
            {"max_vertical_velocity", 1.0},
            {"max_horizontal_velocity", 1.0},
            {"align_descent_velocity", 0.15},
            {"landing_velocity", 0.5},
            {"search_velocity", 0.5},
            {"max_yaw_rate", 0.5},

            // === Tolerâncias ===
            {"position_tolerance", 0.07},
            {"align_tolerance", 0.05},         // Usado pelos align states
            {"align_base_tolerance", 0.05},    // Tolerância específica para bases
            {"align_package_tolerance", 0.02}, // Tolerância específica para packages
            {"yaw_align_tolerance", 0.087},    // ~5 graus para alinhamento rotacional

            // === Timeouts ===
            {"detection_timeout", 10.0},
            {"landing_timeout", 7.0},
            {"garra_timeout", 5.0},
            {"package_operation_timeout", 5.0},

            // === Controladores PID - Posição ===
            {"pid_pos_kp", 1.0},
            {"pid_pos_ki", 0.01},
            {"pid_pos_kd", 0.05},
            {"setpoint", 0.0},

            // === Controladores PID - Yaw (para packages) ===
            {"pid_yaw_kp", 2.0},
            {"pid_yaw_ki", 0.0},
            {"pid_yaw_kd", 0.1},

            // === Offsets de Alinhamento ===
            {"base_align_offset_x", 0.05},     // Offset da câmera para bases
            {"base_align_offset_y", 0.02},
            {"package_align_offset_x", 0.15},  // Offset da garra para packages
            {"package_align_offset_y", 0.0},

            // === Parâmetros de Visão ===
            {"height_to_ground", 1.2},
            {"mean_base_height", 0.0},
            {"mean_package_height", 0.1},
            {"known_base_radius", 1.5},

            // === Busca em Espiral ===
            {"search_radius", 0.5},            // Raio inicial da espiral
            {"angle_increment", 0.5},          // Incremento angular
            {"max_radius", 5.0},               // Raio máximo da espiral

            // === Estados e Flags ===
            {"initial_takeoff_taken", false},   // Flag se já decolou
            {"deliver_after_takeoff", false},   // Flag para entregar após decolagem
            {"package_state", std::string("get_package")}, // "get_package" ou "deliver_package"
            {"package_operation", std::string("pickup")},  // "pickup" ou "drop"
            {"are_there_packages_yet", true},  // Flag se há packages detectados
            {"current_waypoint_index", 0.0},      // Índice do waypoint atual

            // === Outras Configurações ===
            {"descent_velocity", 0.15}          // Velocidade de descida
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

        fsm_ = std::make_unique<Fase2FSM>(drone_node_, vision_node_, parameters);
        
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
