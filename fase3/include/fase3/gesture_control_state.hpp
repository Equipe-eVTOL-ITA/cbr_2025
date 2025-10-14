#pragma once

#include <Eigen/Eigen>
#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

#include "drone/Drone.hpp"
#include "fsm/fsm.hpp"
#include "PidController.hpp"
#include "transformations.hpp"

class GestureControlState : public fsm::State {
public:
    GestureControlState() : fsm::State(),
        yaw_pid_(0.0f, 0.0f, 0.0f, 0.5f),
        climb_pid_(0.0f, 0.0f, 0.0f, 0.5f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        this->drone = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (this->drone == nullptr) {
            return;
        }

        this->drone->log("");
        this->drone->log("STATE: GESTURE CONTROL");

        this->control_speed_ = *blackboard.get<float>("control_speed");
        const float yaw_kp = *blackboard.get<float>("yaw_pid_kp");
        const float yaw_ki = *blackboard.get<float>("yaw_pid_ki");
        const float yaw_kd = *blackboard.get<float>("yaw_pid_kd");
        const float climb_kp = *blackboard.get<float>("climb_pid_kp");
        const float climb_ki = *blackboard.get<float>("climb_pid_ki");
        const float climb_kd = *blackboard.get<float>("climb_pid_kd");

        this->yaw_pid_.setTunings(yaw_kp, yaw_ki, yaw_kd);
        this->climb_pid_.setTunings(climb_kp, climb_ki, climb_kd);
        this->yaw_pid_.setSetpoint(0.5f);
        this->climb_pid_.setSetpoint(0.5f);
        this->yaw_pid_.reset();
        this->climb_pid_.reset();

        this->hand_x_buffer_.clear();
        this->gesture_buffer_.clear();

        this->buffer_size_ = static_cast<std::size_t>(*blackboard.get<float>("gesture_buffer_size"));

        // Debug
        counter = 0;
        this->drone->log("GestureControlState initialized with control speed: " + std::to_string(this->control_speed_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (this->drone == nullptr) {
            return "SEG FAULT";
        }

        gestures = this->drone->getHandGestures();
        const std::array<float, 2> hand_location = this->drone->getHandLocation();

        hand_x = hand_location[0];
        hand_y = hand_location[1];

        this->yaw_rate = this->yaw_pid_.compute(hand_x);
        this->climb_rate = -this->climb_pid_.compute(hand_y);

        this->updateHandXBuffer(hand_x);

        updateHandXBuffer(hand_x);
        if (gestures.size() > 1) {
            updateGestureBuffer(gestures[1]);
        }


        // std::string control_gesture;
        // if (gestures.size() > 1U && !gestures[1].empty()) {
        //     control_gesture = gestures[1];
        // } else if (!gestures.empty() && !gestures[0].empty()) {
        //     control_gesture = gestures[0];
        // }

        // if (!control_gesture.empty()) {
        //     this->updateGestureBuffer(control_gesture);
        // }

        // Debug: Log the control gesture
        // if (counter++ % 5 == 0) {  // ✅ Add this - logs every ~1 second
        //     this->drone->log("control_gesture: '" + control_gesture + "'");
        // }

        if (allElementsEqual(hand_x_buffer_)){
            drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        }
        else {
            if (allElementsEqual(gesture_buffer_, "Thumb_Down")) {
                return "LAND NOW";
            } else if (allElementsEqual(gesture_buffer_, "Open_Palm")) {
                return "GO HOME";
            }

            if (gestures.size() > 1){
                handleGesture(gestures[1]);  // Control the drone based on the second gesture
            } else {
                drone->setLocalVelocity(0.0, 0.0, this->climb_rate, this->yaw_rate);
            }
        }


        // if (this->allElementsEqual(this->hand_x_buffer_)) {
        //     this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        // } else {
        //     if (this->allElementsEqual(this->gesture_buffer_, "Thumb_Down")) {
        //         return "LAND NOW";
        //     }
        //     if (this->allElementsEqual(this->gesture_buffer_, "Open_Palm")) {
        //         return "GO HOME";
        //     }

        //     if (!control_gesture.empty()) {
        //         this->handleGesture(control_gesture, climb_rate, yaw_rate);
        //     } else {
        //         this->drone->setLocalVelocity(0.0, 0.0, climb_rate, yaw_rate);
        //     }
        // }

        usleep(50000); // ~20 Hz
        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (this->drone != nullptr) {
            this->drone->resetHands();
            this->drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
        }
        this->hand_x_buffer_.clear();
        this->gesture_buffer_.clear();
    }

private:

    std::shared_ptr<Drone> drone;
    PidController yaw_pid_;
    PidController climb_pid_;
    float yaw_rate;
    float climb_rate;

    std::deque<float> hand_x_buffer_;
    std::deque<std::string> gesture_buffer_;

    float hand_x;
    float hand_y;

    float control_speed_;
    std::vector<std::string> gestures;
    std::size_t buffer_size_;

    // Debug
    int counter;

    void handleGesture(const std::string &gesture) {

        // Debug
        if (counter++ % 5 == 0){
            this->drone->log("INSIDE HANDLE\n");
        }

        Eigen::Vector3d v;
        Eigen::Vector3d relative_v;

        if (gesture == "Closed_Fist") { 
            v = {-this->control_speed_, 0.0, this->climb_rate}; // Closed Fist -> Backwards
        } else if (gesture == "Pointing_Up") {
            v = {this->control_speed_, 0.0, this->climb_rate}; // Pointing Up -> Forward
        } else if (gesture == "Victory") {
            v = {0.0, this->control_speed_, this->climb_rate}; // Victory -> Right
        } else if (gesture == "ILoveYou") {
            v = {0.0, -this->control_speed_, this->climb_rate}; // I Love You -> Left
        } else {
            v = {0.0, 0.0, this->climb_rate}; // I Love You -> Left
        }
        
        relative_v = adjust_velocity_using_yaw(v, this->drone->getOrientation()[2]);
        drone->setLocalVelocity(relative_v.x(), relative_v.y(), relative_v.z(), this->yaw_rate); 
    }

    // Update HandLocation buffer
    void updateHandXBuffer(float new_hand_x) {
        if (hand_x_buffer_.size() >= 10) {
            hand_x_buffer_.pop_front();
        }
        hand_x_buffer_.push_back(new_hand_x);
    }

    // Update Gesture buffer
    void updateGestureBuffer(const std::string& new_gesture) {
        if (gesture_buffer_.size() >= 10) {
            gesture_buffer_.pop_front();
        }
        gesture_buffer_.push_back(new_gesture);
    }

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
