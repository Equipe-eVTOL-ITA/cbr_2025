#include "fase1/initial_takeoff_state.hpp"
#include "fase1/landing_state.hpp"
#include "fase1/return_home_state.hpp"
#include "fase1/search_bases_state.hpp"
#include "fase1/takeoff_state.hpp"
#include "fase1/visit_bases_state.hpp"
#include "fase1/CoordinateTransforms.hpp"
#include "fase1/Base.hpp"
#include "fase1/ArenaPoint.hpp"
#include <rclcpp/rclcpp.hpp>


#include <memory>
#include <iostream>


class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        Drone* drone = blackboard_get<Drone>("drone");

        const Eigen::Vector3d fictual_home = Eigen::Vector3d({1.2, -1.0, -0.6});
        drone->setHomePosition(fictual_home);

        std::vector<Base> bases;
        const Eigen::Vector3d home_pos = drone->getLocalPosition(); 
        const Eigen::Vector3d orientation = drone->getOrientation();

        bases.push_back({home_pos, true});
        this->blackboard_set<std::vector<Base>>("bases", bases);
        this->blackboard_set<Eigen::Vector3d>("home_position", home_pos);
        this->blackboard_set<bool>("finished_bases", false);
        this->blackboard_set<float>("initial_yaw", orientation[2]);

        // ARENA POINTS
        std::vector<ArenaPoint> waypoints;
        float takeoff_height = -2.3;
        waypoints.push_back({Eigen::Vector3d({1.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({3.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -1.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({5.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -7.0, takeoff_height})});
        waypoints.push_back({Eigen::Vector3d({6.0, -1.0, takeoff_height})});
        this->blackboard_set<std::vector<ArenaPoint>>("waypoints", waypoints);
        this->blackboard_set<float>("takeoff_height", takeoff_height);

        // COORDINATE TRANSFORMS
        float fx = 640.0f;
        float fy = 480.0f;
        float cx = 320.0f;
        float cy = 240.0f;
        float k1 = 0.0f, k2 = 0.0f, k3 = 0.0f, p1 = 0.0f, p2 = 0.0f;
        float ground_z = -1.0;

        // Camera intrinsic matrix K
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            fx, 0, cx,
            0, fy, cy,
            0,  0,  1);

        // Distortion coefficients
        cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
            k1, k2, p1, p2, k3);

        // Camera mounting parameters
        Eigen::Vector3d t_dc(0, 0, 0.2);
        Eigen::Matrix3d R_dc = Eigen::Matrix3d::Identity();

        CoordinateTransforms* coord_transforms = new CoordinateTransforms(
            t_dc, R_dc, camera_matrix, dist_coeffs, ground_z, orientation[2]);

        this->blackboard_set<CoordinateTransforms>("coordinate_transforms", coord_transforms);

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("SEARCH BASES", std::make_unique<SearchBasesState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBasesState>());
        this->add_state("LANDING", std::make_unique<LandingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());

        // Initial Takeoff transitions
        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "SEARCH BASES"},{"SEG FAULT", "ERROR"}});

        // Search Bases transitions
        this->add_transitions("SEARCH BASES", {{"BASES FOUND", "VISIT BASE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("SEARCH BASES", {{"SEARCH ENDED", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Visit Base transitions
        this->add_transitions("VISIT BASE", {{"ARRIVED AT BASE", "LANDING"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("VISIT BASE", {{"LOST BASE", "SEARCH BASES"}, {"SEG FAULT", "ERROR"}});

        // Landing transitions
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});

        // Takeoff transitions
        this->add_transitions("TAKEOFF", {{"NEXT BASE", "SEARCH BASES"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("TAKEOFF", {{"FINISHED BASES", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        // Return Home transitions
        this->add_transitions("RETURN HOME", {{"AT HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});
        
    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase1_node"), my_fsm() {
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
    Fase1FSM my_fsm;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    auto my_node = std::make_shared<NodeFSM>();
    rclcpp::spin(my_node);

    return 0;
}
