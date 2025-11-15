#ifndef THRUST_INTERFACE_HPP
#define THRUST_INTERFACE_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/pwms.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace rclcpp;

class Thrust_Interface : public Node {
    public:
    Thrust_Interface(std::vector<int> thrusters, char* pico_path, int min_pwm, int max_pwm);
    
    private:
    void pwm_received_callback(custom_interfaces::msg::Pwms::UniquePtr pwms_msg);
    void mux_heartbeat_received_callback(std_msgs::msg::Bool::UniquePtr heartbeat);
    void heartbeat_check_callback();
    rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_subscription;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;

    void send_to_pico(int thruster, int pwm);
    int open_pico_serial(char* path);
    int pico_fd = -1;
    std::vector<int> thrusters;
    int min_pwm;
    int max_pwm;
    std::chrono::time_point<std::chrono::steady_clock> most_recent_heartbeat;
    bool no_heartbeat;
};

#endif // THRUST_INTERFACE_HPP