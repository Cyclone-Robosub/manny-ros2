#ifndef MUX_SHERRY_HPP
#define MUX_SHERRY_HPP

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/pwms.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class SoftMux : public rclcpp::Node {
    public:
        SoftMux();
        void pwm_ctrl_callback(custom_interfaces::msg::Pwms::UniquePtr pwm);
        void pwm_cli_callback(custom_interfaces::msg::Pwms::UniquePtr pwm);
        void set_mode_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    private:
        void pwm_cmd_publish(custom_interfaces::msg::Pwms::UniquePtr pwm);
        bool is_matlab_mode;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_mode;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_ctrl_subscriber;
        rclcpp::Subscription<custom_interfaces::msg::Pwms>::SharedPtr pwm_cli_subscriber;
        rclcpp::Publisher<custom_interfaces::msg::Pwms>::SharedPtr pwm_cmd_publisher; 
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr current_control_mode;
};
#endif