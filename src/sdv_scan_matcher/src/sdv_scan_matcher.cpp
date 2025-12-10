#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>

#include <vector>
#include <chrono>
#include <cmath>

using std::vector;
using Eigen::Vector2d;
using Eigen::Matrix2d;
using Eigen::MatrixXd;

class ScanMatcher : public rclcpp::Node
{
public:
  ScanMatcher()
  : Node("sdv_scan_matcher"),
    x_(0.0), y_(0.0), yaw_(0.0)
  {
    // params
    this->declare_parameter<int>("downsample_step", 4);
    this->declare_parameter<int>("max_iterations", 20);
    this->declare_parameter<double>("max_correspondence_dist", 0.2);
    this->get_parameter("downsample_step", downsample_step_);
    this->get_parameter("max_iterations", max_iterations_);
    this->get_parameter("max_correspondence_dist", max_correspondence_dist_);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",  rclcpp::SensorDataQoS(),
      std::bind(&ScanMatcher::scan_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_lidar", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  vector<Vector2d> prev_points_;
  bool have_prev_ = false;

  // pose integrated
  double x_, y_, yaw_;

  // params
  int downsample_step_;
  int max_iterations_;
  double max_correspondence_dist_;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto pts = laser_to_points(msg);
    if (pts.empty()) return;

    if (!have_prev_) {
      prev_points_ = pts;
      have_prev_ = true;
      return;
    }

    // Run ICP (point-to-point) from pts (source) -> prev_points_ (target)
    Eigen::Matrix3d T = Eigen::Matrix3d::Identity(); // homogeneous transform from source to target

    // make mutable copy of source
    vector<Vector2d> src = pts;
    for (int iter = 0; iter < max_iterations_; ++iter) {
      // find correspondences (brute force)
      vector<std::pair<Vector2d, Vector2d>> pairs;
      pairs.reserve(src.size());
      for (const auto &p : src) {
        double best_d2 = max_correspondence_dist_ * max_correspondence_dist_;
        bool found = false;
        Vector2d best_q;
        for (const auto &q : prev_points_) {
          double d2 = (p - q).squaredNorm();
          if (d2 < best_d2) {
            best_d2 = d2;
            best_q = q;
            found = true;
          }
        }
        if (found) pairs.emplace_back(p, best_q);
      }

      if (pairs.size() < 3) break; // not enough correspondences

      // build matrices for SVD solution
      // compute centroids
      Vector2d centroid_p(0,0), centroid_q(0,0);
      for (auto &pr : pairs) {
        centroid_p += pr.first;
        centroid_q += pr.second;
      }
      centroid_p /= double(pairs.size());
      centroid_q /= double(pairs.size());

      // compute W = sum (q_i - centroid_q) * (p_i - centroid_p)^T
      Matrix2d W = Matrix2d::Zero();
      for (auto &pr : pairs) {
        Vector2d p_hat = pr.first - centroid_p;
        Vector2d q_hat = pr.second - centroid_q;
        W += q_hat * p_hat.transpose();
      }

      // SVD
      Eigen::JacobiSVD<Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Matrix2d U = svd.matrixU();
      Matrix2d V = svd.matrixV();
      Matrix2d R = U * V.transpose();

      // ensure proper rotation (det=+1)
      if (R.determinant() < 0) {
        Matrix2d Sv = Matrix2d::Identity();
        Sv(1,1) = -1;
        R = U * Sv * V.transpose();
      }

      Vector2d t = centroid_q - R * centroid_p;

      // form incremental transform
      Eigen::Matrix3d dT = Eigen::Matrix3d::Identity();
      dT(0,0) = R(0,0); dT(0,1) = R(0,1);
      dT(1,0) = R(1,0); dT(1,1) = R(1,1);
      dT(0,2) = t.x(); dT(1,2) = t.y();

      // apply to src (transform points)
      for (auto &p : src) {
        Eigen::Vector3d ph(p.x(), p.y(), 1.0);
        Eigen::Vector3d ph2 = dT * ph;
        p.x() = ph2.x(); p.y() = ph2.y();
      }

      // accumulate T = dT * T
      T = dT * T;

      // small convergence check
      double trans_norm = t.norm();
      double angle = std::atan2(R(1,0), R(0,0));
      if (trans_norm < 1e-4 && std::abs(angle) < 1e-4) break;
    } // ICP iters

    // Extract dx,dy,yaw from T (transform source -> target)
    double dx = T(0,2);
    double dy = T(1,2);
    double dyaw = std::atan2(T(1,0), T(0,0));

    // integrate in odom frame
    double c = std::cos(yaw_), s = std::sin(yaw_);
    x_ += c*dx - s*dy;
    y_ += s*dx + c*dy;
    yaw_ += dyaw;

    // publish odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    // quaternion from yaw
    double qz = std::sin(yaw_/2.0);
    double qw = std::cos(yaw_/2.0);
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;

    odom_pub_->publish(odom);

    // broadcast TF
    geometry_msgs::msg::TransformStamped tmsg;
    tmsg.header.stamp = msg->header.stamp;
    tmsg.header.frame_id = "odom";
    tmsg.child_frame_id = "base_link";
    tmsg.transform.translation.x = x_;
    tmsg.transform.translation.y = y_;
    tmsg.transform.translation.z = 0.0;
    tmsg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tmsg);

    // set prev = current (optionally keep a copy or a moving window)
    prev_points_ = pts;
  }

  vector<Vector2d> laser_to_points(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    vector<Vector2d> pts;
    int N = static_cast<int>(std::floor((msg->angle_max - msg->angle_min) / msg->angle_increment)) + 1;
    pts.reserve(N / std::max(1, downsample_step_));

    for (int i = 0; i < N; i += downsample_step_) {
      double r = msg->ranges[i];
      if (!std::isfinite(r)) continue;
      if (r <= msg->range_min || r >= msg->range_max) continue;
      double angle = msg->angle_min + i * msg->angle_increment;
      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      pts.emplace_back(x, y);
    }
    return pts;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanMatcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
