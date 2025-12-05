#include <memory>
#include <queue>
#include <vector>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace sdv_planner
{

struct GraphNode
{
    int x, y;
    double cost = 0;
    double heuristic = 0;
    std::shared_ptr<GraphNode> prev = nullptr;

    GraphNode(int x_=0, int y_=0) : x(x_), y(y_) {}
    bool operator==(const GraphNode &other) const { return x == other.x && y == other.y; }
    bool operator>(const GraphNode &other) const { return (cost + heuristic) > (other.cost + other.heuristic); }
    GraphNode operator+(const std::pair<int,int> &dir) const { return GraphNode(x+dir.first, y+dir.second); }
};

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
    AStarPlanner() = default;

    void configure(
        const rclcpp::Node::SharedPtr &parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> &tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros) override
    {
        node_ = parent;
        name_ = name;
        tf_buffer_ = tf;
        costmap_ros_ = costmap_ros;

        RCLCPP_INFO(node_->get_logger(), "AStarPlanner plugin configured.");
    }

    void cleanup() override {}
    void activate() override {}
    void deactivate() override {}

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override
    {
        // Convert start and goal to map frame
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";

        // For demo purposes, path is just straight line (replace con tu A*)
        path.poses.push_back(start);
        path.poses.push_back(goal);

        RCLCPP_INFO(node_->get_logger(), "createPlan called (replace with A*).");

        return path;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

}  // namespace sdv_planner

PLUGINLIB_EXPORT_CLASS(sdv_planner::AStarPlanner, nav2_core::GlobalPlanner)
