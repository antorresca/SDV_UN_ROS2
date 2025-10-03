#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sdv_controller/motor.h"

class SdvControllerNode : public rclcpp::Node{
    public: SdvControllerNode() : Node("sdv_controller"){
        RCLCPP_INFO(this->get_logger(), "Nodo 'sdv_controller' ejecutandose");
        pub_ = this->create_publisher<std_msgs::msg::String>("/vel2cmd",10);

        // Ejemplo: publicar cada 500ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SdvControllerNode::publish_message, this));

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&SdvControllerNode::topic_callback, this, std::placeholders::_1));

        motor_left  = std::make_shared<Motor>(0.075, false);
        motor_right = std::make_shared<Motor>(0.075, true); 
        motor_left->setWheelSeparation(0.44010); 
        motor_right->setWheelSeparation(0.44010);

    }

    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

    std::shared_ptr<Motor> motor_left;
    std::shared_ptr<Motor> motor_right;

    double PWM_R;
    double PWM_L;

    void publish_message(){
        std_msgs::msg::String msg;
        msg.data = "m 1 "+std::to_string((int)PWM_R)+" "+std::to_string((int)PWM_L)+" \r";
        RCLCPP_INFO(this->get_logger(), "Publicando '%s'", msg.data.c_str());
        pub_->publish(msg);
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        double vx = msg->linear.x;     // m/s
        double wz = msg->angular.z;    // rad/s

        rclcpp::Clock clock;  // Para timestamp de Motor
        PWM_R = motor_right->getPwmPercent(vx, wz, clock);
        PWM_L = motor_left->getPwmPercent(vx, wz, clock);

        PWM_L = std::clamp(PWM_L,-40.0,40.0);
        PWM_R = std::clamp(PWM_R,-40.0,40.0);

        RCLCPP_INFO(this->get_logger(), "PWM R=%.2f, L=%.2f", PWM_R, PWM_L);    
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SdvControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
