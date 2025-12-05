#include <chrono>

#include "sdv_tracking/pure_pursuit.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sdv_tracking
{
PurePursuit::PurePursuit() : Node("pure_pursuit_motion_planner_node"),
    look_ahead_distance_(0.2), max_linear_velocity_(0.2), max_angular_velocity_(0.2)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
  declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
  declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
  look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/a_star/path", 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));
        
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  carrot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot", 10);
  control_loop_ = create_wall_timer(
    std::chrono::milliseconds(250), std::bind(&PurePursuit::controlLoop, this));
}

void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  global_plan_ = *path;
}

void PurePursuit::controlLoop()
{
  if(global_plan_.poses.empty()){
    return;
  }

  // Get the robot's current pose in the odom frame
  geometry_msgs::msg::TransformStamped robot_pose;
  try {
    robot_pose = tf_buffer_->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  if(!transformPlan(robot_pose.header.frame_id)){
    RCLCPP_ERROR(get_logger(), "Unable to transform Plan in robot's frame");
    return;
  }

  geometry_msgs::msg::PoseStamped robot_pose_stamped;
  robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
  robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
  robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
  robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

  auto carrot_pose = getCarrotPose(robot_pose_stamped);

  // compute vector to carrot in robot frame
  double dx = carrot_pose.pose.position.x - robot_pose_stamped.pose.position.x;
  double dy = carrot_pose.pose.position.y - robot_pose_stamped.pose.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // goal reached
  if(distance <= 0.1){
      RCLCPP_INFO(get_logger(), "Goal Reached!");

      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_pub_->publish(stop_cmd);

      global_plan_.poses.clear();
      return;
  }

  carrot_pub_->publish(carrot_pose);

  // Transform carrot into robot frame (so carrot_pose.pose.x,y are relative)
  tf2::Transform robot_tf, carrot_tf, carrot_in_robot_tf;
  tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
  tf2::fromMsg(carrot_pose.pose, carrot_tf);
  carrot_in_robot_tf = robot_tf.inverse() * carrot_tf;
  geometry_msgs::msg::Pose carrot_in_robot_msg;
  tf2::toMsg(carrot_in_robot_tf, carrot_in_robot_msg);

  double cx = carrot_in_robot_msg.position.x;
  double cy = carrot_in_robot_msg.position.y;

  // if carrot is behind robot, slow down and turn in place if necessary
  if (cx < 0.0) {
    // carrot behind -> reduce speed to allow rotation or select goal directly
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    double yaw_error = std::atan2(cy, cx);
    double ang = std::clamp(2.0 * yaw_error, -max_angular_velocity_, max_angular_velocity_);
    cmd_vel.angular.z = ang;
    cmd_pub_->publish(cmd_vel);
    return;
  }

  // Pure pursuit curvature and alternative: use heading error alpha
  double alpha = std::atan2(cy, cx);              // heading to carrot in robot frame
  double curvature = getCurvature(carrot_in_robot_msg);

  // compute linear speed scaled by heading error (slower on sharper turns)
  double heading_scale = std::max(0.0, 1.0 - std::min(std::abs(alpha) / (M_PI/4.0), 1.0)); // reduces speed if |alpha| > 45deg
  double linear = max_linear_velocity_ * heading_scale;

  // prefer angular = linear * curvature (classical pure pursuit)
  double angular = linear * curvature;

  // clamp
  if (linear > max_linear_velocity_) linear = max_linear_velocity_;
  if (linear < 0.0) linear = 0.0;
  angular = std::clamp(angular, -max_angular_velocity_, max_angular_velocity_);

  // publish
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;
  cmd_pub_->publish(cmd_vel);
}

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  // If plan empty, return last
  if (global_plan_.poses.empty()) {
    geometry_msgs::msg::PoseStamped empty;
    return empty;
  }

  // 1) find nearest index on the path to the robot
  size_t nearest_idx = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double d = std::hypot(dx, dy);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest_idx = i;
    }
  }

  // 2) from nearest index, advance until we find the first point with distance >= look_ahead_distance_
  for (size_t i = nearest_idx; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - robot_pose.pose.position.x;
    double dy = global_plan_.poses[i].pose.position.y - robot_pose.pose.position.y;
    double d = std::hypot(dx, dy);
    if (d >= look_ahead_distance_) {
      return global_plan_.poses[i];
    }
  }

  // 3) if none found, return the final goal
  return global_plan_.poses.back();
}


double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & carrot_pose)
{
  const double carrot_dist =
  (carrot_pose.position.x * carrot_pose.position.x) +
  (carrot_pose.position.y * carrot_pose.position.y);
    
  // Find curvature of circle (k = 1 / R)
  if (carrot_dist > 0.001) {
      return 2.0 * carrot_pose.position.y / carrot_dist;
  } else {
      return 0.0;
  }
}

bool PurePursuit::transformPlan(const std::string & frame)
{
  if(global_plan_.header.frame_id == frame){
    return true;
  }
  geometry_msgs::msg::TransformStamped transform;
  try{
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " <<
      global_plan_.header.frame_id << " to frame " << frame);
    return false;
  }
  for(auto & pose : global_plan_.poses){
    tf2::doTransform(pose, pose, transform);
  }
  global_plan_.header.frame_id = frame;
  return true;
}
}  // namespace bumperbot_motion

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sdv_tracking::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}