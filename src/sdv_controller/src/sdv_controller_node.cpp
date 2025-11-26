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

        // Publicar cada 100ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SdvControllerNode::publish_message, this));

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&SdvControllerNode::topic_callback, this, std::placeholders::_1));

    }

    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;

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

        PWM_R = getPWM(vx,wz,true);
        PWM_L = getPWM(vx,wz,false);

        PWM_L = std::clamp(PWM_L,-40.0,40.0);
        PWM_R = std::clamp(PWM_R,-40.0,40.0);

        RCLCPP_INFO(this->get_logger(), "PWM R=%.2f, L=%.2f", PWM_R, PWM_L);    
    }

    int getPWM(double V, double W,bool Side){
        double wheel_radio = 0.075; //m
        double wheel_base = 0.32;  //m
        int factor = 0;

        double w_wheel =  V/wheel_radio;

        if(Side){ //If Side == True, it's right wheel
            w_wheel -= (wheel_base*W)/wheel_radio;
        }else{
            w_wheel += (wheel_base*W)/wheel_radio;
        };

        if(w_wheel>0){
        factor = 1;
        }else{
        factor = -1;
        };

        return (0.8333*(30/3.141592)*abs(w_wheel)+10)*factor;
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
