#include "initial_takeoff_state.hpp"
#include "landing_state.hpp"
#include "return_home_state.hpp"
#include "search_state.hpp"
#include "takeoff_state.hpp"
#include "gesture_control_state.hpp"
#include <rclcpp/rclcpp.hpp>
 
#include <memory>
#include <iostream>


class Fase4FSM : public fsm::FSM {
public:
    Fase4FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.0, -0.6});
        drone->setHomePosition(fictual_home);

        const Eigen::Vector3d home_pos = drone->getLocalPosition();
        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);

        this->blackboard_set<float>("takeoff height", -1.6);
        this->blackboard_set<float>("control speed", 0.4);
        this->blackboard_set<float>("yaw speed", 0.4);


        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH", std::make_unique<SearchState>());
        this->add_state("GESTURE CONTROL", std::make_unique<GestureControlState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH"},{"SEG FAULT", "ERROR"}});

        // Search transitions
        this->add_transitions("SEARCH", {{"HAND FOUND", "GESTURE CONTROL"},{"SEG FAULT", "ERROR"}});

        // Gesture Control transitions
        this->add_transitions("GESTURE CONTROL", {{"LAND NOW", "LANDING"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("GESTURE CONTROL", {{"GO HOME", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});

        // Takeoff transitions
        this->add_transitions("TAKEOFF", {{"TAKEOFF COMPLETED", "GESTURE CONTROL"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase4_node"), my_fsm() {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Run at approximately 20 Hz
            std::bind(&NodeFSM::executeFSM, this));
    }

    void executeFSM() {
        if (rclcpp::ok() && !my_fsm.is_finished()) {
            my_fsm.execute();
        } else {
            rclcpp::shutdown();
        }
    }

private:
    Fase4FSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}


