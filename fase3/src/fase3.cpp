#include <chrono>
#include <functional>
#include <memory>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"

#include "fase3/gesture_control_state.hpp"
#include "fase3/initial_takeoff_state.hpp"
#include "fase3/landing_state.hpp"
#include "fase3/return_home_state.hpp"
#include "fase3/search_state.hpp"
#include "fase3/takeoff_state.hpp"

using ParameterMap = std::unordered_map<std::string, double>;

class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM(std::shared_ptr<Drone> drone,
             const ParameterMap &params)
        : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        for (const auto &[key, value] : params) {
            this->blackboard_set<float>(key, static_cast<float>(value));
        }

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH", std::make_unique<SearchState>());
        this->add_state("GESTURE CONTROL", std::make_unique<GestureControlState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        this->add_transitions("INITIAL TAKEOFF", {
            {"INITIAL TAKEOFF COMPLETED", "SEARCH"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("SEARCH", {
            {"HAND FOUND", "GESTURE CONTROL"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("GESTURE CONTROL", {
            {"LAND NOW", "LANDING"},
            {"GO HOME", "RETURN HOME"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "TAKEOFF"},
            {"SEG FAULT", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "GESTURE CONTROL"},
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
    explicit NodeFSM(std::shared_ptr<Drone> drone)
        : rclcpp::Node("cbr_fase3_fsm"), drone_node_(std::move(drone)) {

        const ParameterMap defaults = {
            {"fictual_home_x", 1.2},
            {"fictual_home_y", -1.0},
            {"fictual_home_z", -0.6},
            {"takeoff_height", -1.8},
            {"return_home_timeout", 30.0},
            {"max_vertical_velocity", 1.0},
            {"max_horizontal_velocity", 1.0},
            {"position_tolerance", 0.05},
            {"control_speed", 0.4},
            {"yaw_speed", 0.35},
            {"search_yaw_range", 1.0472},
            {"gesture_buffer_size", 10},
            {"landing_velocity_max", 0.5},
            {"landing_velocity_min", 0.2},
            {"max_base_height", -0.2},
            {"landing_timeout", 5.0},
            {"yaw_pid_kp", 0.6},
            {"yaw_pid_ki", 0.0},
            {"yaw_pid_kd", 0.06},
            {"climb_pid_kp", 0.9},
            {"climb_pid_ki", 0.0},
            {"climb_pid_kd", 0.09}
        };

        const ParameterMap params = declareAndGetParameters(defaults);
        fsm_ = std::make_unique<Fase3FSM>(drone_node_, params);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&NodeFSM::executeFSM, this));
    }

private:
    ParameterMap declareAndGetParameters(const ParameterMap &defaults) {
        ParameterMap result;
        for (const auto &[name, value] : defaults) {
            this->declare_parameter(name, value);
            result[name] = this->get_parameter(name).as_double();
        }
        return result;
    }

    void executeFSM() {
        if (!rclcpp::ok()) {
            return;
        }

        if (fsm_ != nullptr && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "Mission finished, shutting down");
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<Drone> drone_node_;
    std::unique_ptr<Fase3FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto drone = std::make_shared<Drone>();
    auto fsm_node = std::make_shared<NodeFSM>(drone);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(drone);
    executor.add_node(fsm_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}