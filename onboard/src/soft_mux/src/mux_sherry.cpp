#include "mux_sherry.hpp"

SoftMux::SoftMux() : rclcpp::Node("SoftMux") {
    pwm_cmd_publisher = this->create_publisher<custom_interfaces::msg::Pwms>("pwm_cmd", 10);
    pwm_ctrl_subscriber = this->create_subscription<custom_interfaces::msg::Pwms>("pwm_ctrl", 10, std::bind(&SoftMux::pwm_ctrl_callback, this, std::placeholders::_1));
    pwm_cli_subscriber =  this->create_subscription<custom_interfaces::msg::Pwms>("pwm_cli", 10, std::bind(&SoftMux::pwm_cli_callback, this, std::placeholders::_1));
    current_control_mode = this->create_publisher<std_msgs::msg::Bool>("current_mode", 10);
    control_mode = this->create_service<std_srvs::srv::SetBool>("control_mode", std::bind(&SoftMux::set_mode_srv, this, std::placeholders::_1, std::placeholders::_2));
}

void SoftMux::pwm_ctrl_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    if (is_matlab_mode) {
        pwm_cmd_publish(std::move(pwm));
    }
}

void SoftMux::pwm_cli_callback(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    if (!is_matlab_mode) {
        pwm_cmd_publish(std::move(pwm));
    }

}

void SoftMux::set_mode_srv(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    this->is_matlab_mode = request->data;
    if (this->is_matlab_mode == request->data) {
        std_msgs::msg::Bool message;
        message.data = this->is_matlab_mode;
        this->current_control_mode->publish(message);
        std::cout << message.data << std::endl;
        response->success = true;
    } else {
        response->success = false;
    }
}

void SoftMux::pwm_cmd_publish(custom_interfaces::msg::Pwms::UniquePtr pwm) {
    this->pwm_cmd_publisher->publish(*(std::move(pwm)));
     for (int i = 0; i < 8; i++) {
         std::cout << "PWM " << i << " is sent as " << pwm->pwms[i] << "\n";
    }

}
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SoftMux>());
    rclcpp::shutdown();
    return 0;
}
