#include "fase2/initial_takeoff_state.hpp"
#include "fase2/return_home_state.hpp"
#include "fase2/search_bases_state.hpp"
#include "fase2/visit_bases_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


class Fase2FSM : public fsm::FSM {
public:
    Fase2FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.0, -0.6});
        drone->setHomePosition(fictual_home);

        const Eigen::Vector3d home_pos = drone->getLocalPosition(); 
        const Eigen::Vector3d orientation = drone->getOrientation();

        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);
        this->blackboard_set<float>("initial_yaw", orientation[2]);

        float takeoff_height = -2.6;
        this->blackboard_set<float>("takeoff_height", takeoff_height);

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASES", std::make_unique<SearchBasesState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBasesState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH BASES"},{"SEG FAULT", "ERROR"}});

        // Search Bases transitions
        this->add_transitions("SEARCH BASES", {{"SEARCH ENDED", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Visit Base transitions
        this->add_transitions("VISIT BASE", {{"FINISHED QR CODES", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase2_node"), my_fsm() {
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
    Fase2FSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
