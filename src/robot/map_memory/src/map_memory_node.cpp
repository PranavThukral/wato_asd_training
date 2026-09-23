#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  const auto resolution = declare_parameter<double>("resolution", 0.10);
  const auto width = declare_parameter<int>("width", 400);
  const auto height = declare_parameter<int>("height", 400);
  const auto origin_x = declare_parameter<double>("origin_x", -20.0);
  const auto origin_y = declare_parameter<double>("origin_y", -20.0);
  const auto update_period = declare_parameter<double>("update_period", 0.20);
  if (!(resolution > 0.0) || width <= 0 || height <= 0 || !(update_period > 0.0)) {
    throw std::invalid_argument("Invalid map memory parameters");
  }
  map_memory_.configure(resolution, width, height, origin_x, origin_y);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", rclcpp::SensorDataQoS(),
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(update_period)), std::bind(&MapMemoryNode::publishMap, this));
  publishMap();
}

void MapMemoryNode::costmapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (msg && !msg->data.empty()) latest_costmap_ = std::move(msg);
}

void MapMemoryNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg && std::isfinite(msg->pose.pose.position.x) && std::isfinite(msg->pose.pose.position.y) &&
      std::isfinite(msg->pose.pose.orientation.x) && std::isfinite(msg->pose.pose.orientation.y) &&
      std::isfinite(msg->pose.pose.orientation.z) && std::isfinite(msg->pose.pose.orientation.w) &&
      msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
      msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
      msg->pose.pose.orientation.w * msg->pose.pose.orientation.w > 1e-8) {
    latest_odom_ = std::move(msg);
    odom_history_.push_back(latest_odom_);
    while (odom_history_.size() > 100) odom_history_.pop_front();
    if (map_memory_.map().header.frame_id.empty()) map_memory_.setFrame(latest_odom_->header.frame_id);
  }
}

void MapMemoryNode::publishMap()
{
  if (latest_costmap_ && !odom_history_.empty() &&
      (!have_fused_stamp_ || rclcpp::Time(latest_costmap_->header.stamp) != last_fused_stamp_)) {
    const rclcpp::Time scan_time(latest_costmap_->header.stamp);
    auto best_odom = odom_history_.front();
    auto best_delta = std::abs((rclcpp::Time(best_odom->header.stamp) - scan_time).seconds());
    for (const auto& candidate : odom_history_) {
      const auto delta = std::abs((rclcpp::Time(candidate->header.stamp) - scan_time).seconds());
      if (delta < best_delta) { best_delta = delta; best_odom = candidate; }
    }
    if (best_delta <= 0.25) {
      bool fused = false;
      if (latest_costmap_->header.frame_id.empty() || best_odom->child_frame_id.empty() ||
          latest_costmap_->header.frame_id == best_odom->child_frame_id) {
        fused = map_memory_.update(*latest_costmap_, best_odom->pose.pose, scan_time);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Dropping costmap frame '%s'; odometry pose is in '%s'",
          latest_costmap_->header.frame_id.c_str(), best_odom->child_frame_id.c_str());
      }
      if (fused) {
        last_fused_stamp_ = scan_time;
        have_fused_stamp_ = true;
      }
    }
  }
  auto map = map_memory_.map();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
