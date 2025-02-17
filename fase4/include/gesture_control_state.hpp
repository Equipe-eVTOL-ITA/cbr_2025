#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <deque>
#include <iostream>
#include <string>
#include <array>
#include "pid_controller.hpp"

class GestureControlState : public fsm::State {
public:
    GestureControlState() : fsm::State(), yaw_pid(0.6, 0.0, 0.06, 0.5), climb_pid(0.9, 0.0, 0.09, 0.5) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;

        drone->log("STATE: GESTURE CONTROL");

        this->control_speed = *blackboard.get<float>("control speed");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        gestures = drone->getHandGestures();
        std::array<float, 2> hand_location = drone->getHandLocation();
        hand_x = hand_location[0];
        hand_y = hand_location[1];

        this->yaw_rate = yaw_pid.compute(hand_x);
        this->climb_rate = -climb_pid.compute(hand_y);
        
        //drone->log("yaw_rate: " + std::to_string(yaw_rate) + " climb_rate: " + std::to_string(climb_rate));

        // Update buffers
        updateHandXBuffer(hand_x);
        if (gestures.size() > 1) {
            updateGestureBuffer(gestures[1]);
        }

        if (allElementsEqual(hand_x_buffer)){
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        }
        else {
            if (allElementsEqual(gesture_buffer, "Thumb_Down")) {
                return "LAND NOW";
            } else if (allElementsEqual(gesture_buffer, "Open_Palm")) {
                return "GO HOME";
            }

            if (gestures.size() > 1){
                handleGesture(gestures[1]);  // Control the drone based on the second gesture
            } else {
                drone->setLocalVelocity(0.0, 0.0, this->climb_rate, this->yaw_rate);
            }
        }

        usleep(5e04);  // Control frequency to approximately 20Hz
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void) blackboard;

        drone->resetHands();
        hand_x_buffer.clear();
        gesture_buffer.clear();
    }


private:
    PidController yaw_pid;
    PidController climb_pid;
    float yaw_rate = 0.0;
    float climb_rate = 0.0;

    std::deque<float> hand_x_buffer;
    std::deque<std::string> gesture_buffer;

    float hand_x = 0.5;
    float hand_y = 0.5;
    float control_speed;

    Drone* drone;
    Eigen::Vector3d pos;
    std::vector<std::string> gestures;

    // Handle gesture-based movements
    void handleGesture(const std::string& gesture) {
        if (gesture == "Closed_Fist") {
            drone->setLocalVelocity(-this->control_speed, 0.0, this->climb_rate, this->yaw_rate); // Closed Fist -> Backward
        } else if (gesture == "Pointing_Up") {
            drone->setLocalVelocity(this->control_speed, 0.0, this->climb_rate, this->yaw_rate); // Pointing Up -> Forward
        } else if (gesture == "Victory") {
            drone->setLocalVelocity(0.0, this->control_speed, this->climb_rate, this->yaw_rate); // Victory -> Right
        } else if (gesture == "ILoveYou") {
            drone->setLocalVelocity(0.0, -this->control_speed, this->climb_rate, this->yaw_rate); // I Love You -> Left
        } else {
            drone->setLocalVelocity(0.0, 0.0, this->climb_rate, this->yaw_rate); // Other gestures -> Follow 1st hand
        }
    }

    // Update HandLocation buffer
    void updateHandXBuffer(float new_hand_x) {
        if (hand_x_buffer.size() >= 10) {
            hand_x_buffer.pop_front();
        }
        hand_x_buffer.push_back(new_hand_x);
    }

    // Update Gesture buffer
    void updateGestureBuffer(const std::string& new_gesture) {
        if (gesture_buffer.size() >= 10) {
            gesture_buffer.pop_front();
        }
        gesture_buffer.push_back(new_gesture);
    }

    // Check if all elements in buffer are equal
    bool allElementsEqual(const std::deque<float>& buffer) {
        if (buffer.empty()) return true;
        float first_value = buffer[0];
        for (const auto& value : buffer) {
            if (value != first_value) {
                return false;
            }
        }
        return buffer.size() == 10;
    }

    // Overload of allElementsEqual for the gesture buffer
    bool allElementsEqual(const std::deque<std::string>& buffer, const std::string& target) {
        if (buffer.empty()) return false;
        for (const auto& value : buffer) {
            if (value != target) {
                return false;
            }
        }
        return buffer.size() == 10;
    }
};
