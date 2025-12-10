#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PurePursuitSLAM : public rclcpp::Node
{
public:
    PurePursuitSLAM() : Node("pure_pursuit_slam_node"),
        look_ahead_distance_(0.3),
        max_linear_velocity_(0.15),
        max_angular_velocity_(0.3),
        pose_received_(false)
    {
        declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
        declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
        declare_parameter<double>("max_angular_velocity", max_angular_velocity_);

        look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
        max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

        // SLAM Toolbox pose
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&PurePursuitSLAM::poseCallback, this, std::placeholders::_1));

        // Global plan
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/a_star/path", 10,
            std::bind(&PurePursuitSLAM::pathCallback, this, std::placeholders::_1));

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        carrot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot", 10);

        control_loop_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PurePursuitSLAM::controlLoop, this));
    }

private:

    // ----------------------------------------------------------
    //   CALLBACK: POSE
    // ----------------------------------------------------------
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convertir PoseWithCovarianceStamped → PoseStamped
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;

        pose_received_ = true;
    }

    // ----------------------------------------------------------
    //   CALLBACK: PATH
    // ----------------------------------------------------------
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        global_plan_ = *msg;
        global_plan_.header.frame_id = "map"; // asegurar consistencia
    }

    // ----------------------------------------------------------
    //   CONTROL LOOP
    // ----------------------------------------------------------
    void controlLoop()
    {
        if (!pose_received_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Esperando pose de SLAM...");
            return;
        }

        if (global_plan_.poses.empty()) {
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose = current_pose_;

        // Obtener el punto carrot
        geometry_msgs::msg::PoseStamped carrot_pose =
            getCarrotPose(robot_pose);

        carrot_pub_->publish(carrot_pose);

        double dx = carrot_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = carrot_pose.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if (distance < 0.15) {
            RCLCPP_INFO(get_logger(), "Meta alcanzada. Deteniendo robot.");
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);
            global_plan_.poses.clear();
            return;
        }

        double curvature = computeCurvature(robot_pose, carrot_pose);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = max_linear_velocity_;
        cmd.angular.z = curvature * max_angular_velocity_;

        cmd_pub_->publish(cmd);
    }

    // ----------------------------------------------------------
    //   PURE PURSUIT CORE
    // ----------------------------------------------------------
    geometry_msgs::msg::PoseStamped getCarrotPose(
        const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        geometry_msgs::msg::PoseStamped carrot =
            global_plan_.poses.back();

        for (auto it = global_plan_.poses.rbegin();
             it != global_plan_.poses.rend(); ++it)
        {
            double dx = it->pose.position.x - robot_pose.pose.position.x;
            double dy = it->pose.position.y - robot_pose.pose.position.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist > look_ahead_distance_) {
                carrot = *it;
            } else {
                break;
            }
        }
        return carrot;
    }

    double computeCurvature(
        const geometry_msgs::msg::PoseStamped &robot,
        const geometry_msgs::msg::PoseStamped &carrot)
    {
        tf2::Transform robot_tf, carrot_tf;
        tf2::fromMsg(robot.pose, robot_tf);
        tf2::fromMsg(carrot.pose, carrot_tf);

        tf2::Transform carrot_in_robot = robot_tf.inverse() * carrot_tf;

        double x = carrot_in_robot.getOrigin().x();
        double y = carrot_in_robot.getOrigin().y();

        double dist2 = x*x + y*y;
        if (dist2 < 1e-4) return 0.0;

        return 2.0 * y / dist2;
    }

    // ----------------------------------------------------------
    //   VARIABLES
    // ----------------------------------------------------------

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pub_;

    rclcpp::TimerBase::SharedPtr control_loop_;

    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::Path global_plan_;

    bool pose_received_;

    double look_ahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitSLAM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
