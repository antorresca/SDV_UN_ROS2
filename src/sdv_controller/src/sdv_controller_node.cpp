#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class SdvControllerNode : public rclcpp::Node {
public:
    SdvControllerNode() : Node("sdv_controller"),
                          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) 
    {
        RCLCPP_INFO(this->get_logger(), "Nodo 'sdv_controller' ejecutándose");

        pub_motor_ = this->create_publisher<std_msgs::msg::String>("/vel2cmd", 10);
        pub_odom_  = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SdvControllerNode::update_odometry, this));

        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&SdvControllerNode::cmd_callback, this, std::placeholders::_1));

        last_time_ = this->now();

        // Inicialización pose
        x_ = 0.0;
        y_ = 0.0;
        th_ = 0.0;

        vx_ = 0.0;
        wz_ = 0.0;
    }

private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_motor_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Odometría interna
    double x_, y_, th_;
    double vx_, wz_;
    rclcpp::Time last_time_;

    // Parametros robot
    double wheel_radius = 0.075;
    double wheel_base   = 0.32;

    // PWM
    double PWM_R, PWM_L;

    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        wz_ = msg->angular.z;

        PWM_R = vx_/wheel_radius + (wheel_base*wz_)/wheel_radius;
        PWM_L = vx_/wheel_radius - (wheel_base*wz_)/wheel_radius;

        PWM_L = std::clamp(PWM_L, -40.0, 40.0);
        PWM_R = std::clamp(PWM_R, -40.0, 40.0);

        std_msgs::msg::String motor_msg;
        motor_msg.data = "m 1 " + std::to_string((int)PWM_R) + " " + std::to_string((int)PWM_L);
        pub_motor_->publish(motor_msg);
    }

    void update_odometry() {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        // Integración odometría
        x_  += vx_ * cos(th_) * dt;
        y_  += vx_ * sin(th_) * dt;
        th_ += wz_ * dt;

        // Publicar TF: odom → base_link
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = now;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id  = "base_link";

        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, th_);
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);

        // Publicar mensaje Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x  = vx_;
        odom_msg.twist.twist.angular.z = wz_;

        pub_odom_->publish(odom_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SdvControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
