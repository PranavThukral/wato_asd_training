#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include "planner_node.hpp"

namespace
{

builtin_interfaces::msg::Time toMessageTime(const rclcpp::Time& time)
{
  builtin_interfaces::msg::Time message;
  auto nanoseconds = time.nanoseconds();
  auto seconds = nanoseconds / 1000000000LL;
  auto remainder = nanoseconds % 1000000000LL;
  if (remainder < 0) {
    --seconds;
    remainder += 1000000000LL;
  }
  message.sec = static_cast<std::int32_t>(seconds);
  message.nanosec = static_cast<std::uint32_t>(remainder);
  return message;
}

}

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger()))
{
  const auto inflation = declare_parameter<double>("inflation_radius", 1.95);
  const auto unknown_penalty = declare_parameter<double>("unknown_penalty", 2.0);
  const auto goal_tolerance = declare_parameter<double>("goal_tolerance", 0.20);
  map_timeout_ = declare_parameter<double>("map_timeout", 0.75);
  if (!(inflation >= 0.0) || !(unknown_penalty >= 0.0) || !(goal_tolerance > 0.0) || !(map_timeout_ > 0.0)) {
    throw std::invalid_argument("Invalid planner parameters");
  }
  planner_.configure(inflation, unknown_penalty, goal_tolerance);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/path", rclcpp::QoS(1).reliable());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  pose_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10, std::bind(&PlannerNode::poseGoalCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
  publishEmptyPath();
}

void PlannerNode::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->info.width == 0 || msg->info.height == 0 ||
      msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height) return;
  map_ = std::move(msg);
  last_map_stamp_ = rclcpp::Time(map_->header.stamp);
  last_map_receive_ = std::chrono::steady_clock::now();
  dirty_ = true;
}

void PlannerNode::goalCallback(geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  if (!msg || !std::isfinite(msg->point.x) || !std::isfinite(msg->point.y)) return;
  goal_ = *msg;
  goal_active_ = true;
  dirty_ = true;
  planIfReady();
}

void PlannerNode::poseGoalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!msg || !std::isfinite(msg->pose.position.x) || !std::isfinite(msg->pose.position.y)) return;
  auto point = std::make_shared<geometry_msgs::msg::PointStamped>();
  point->header = msg->header;
  point->point = msg->pose.position;
  goalCallback(std::move(point));
}

void PlannerNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg && std::isfinite(msg->pose.pose.position.x) && std::isfinite(msg->pose.pose.position.y) &&
      std::isfinite(msg->pose.pose.orientation.x) && std::isfinite(msg->pose.pose.orientation.y) &&
      std::isfinite(msg->pose.pose.orientation.z) && std::isfinite(msg->pose.pose.orientation.w) &&
      msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
      msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
      msg->pose.pose.orientation.w * msg->pose.pose.orientation.w > 1e-8) {
    odom_ = std::move(msg);
  }
}

void PlannerNode::publishEmptyPath()
{
  nav_msgs::msg::Path path;
  if (map_) path.header = map_->header;
  path.header.stamp = toMessageTime(get_clock()->now());
  active_path_ = nav_msgs::msg::Path();
  have_active_path_ = false;
  path_pub_->publish(path);
}

void PlannerNode::planIfReady()
{
  if (!goal_active_) return;
  const auto steady_now = std::chrono::steady_clock::now();
  if (last_map_receive_.time_since_epoch().count() == 0 ||
      std::chrono::duration<double>(steady_now - last_map_receive_).count() > map_timeout_) {
    publishEmptyPath();
    dirty_ = false;
    return;
  }
  if (last_map_stamp_.nanoseconds() == 0 || get_clock()->now().nanoseconds() > 0 &&
      ((get_clock()->now() - last_map_stamp_).seconds() > map_timeout_ ||
       (get_clock()->now() - last_map_stamp_).seconds() < -0.25)) {
    publishEmptyPath();
    dirty_ = false;
    return;
  }
  if (!dirty_ && have_active_path_) {
    auto heartbeat = active_path_;
    heartbeat.header.stamp = toMessageTime(get_clock()->now());
    path_pub_->publish(heartbeat);
    return;
  }
  if (!map_ || !odom_ || map_->data.empty()) return;
  if (std::none_of(map_->data.begin(), map_->data.end(), [](const auto value) { return value >= 0; })) {
    publishEmptyPath();
    dirty_ = false;
    return;
  }
  if (!goal_.header.frame_id.empty() && goal_.header.frame_id != map_->header.frame_id) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Goal frame '%s' does not match map frame '%s'", goal_.header.frame_id.c_str(), map_->header.frame_id.c_str());
    publishEmptyPath();
    dirty_ = false;
    return;
  }
  nav_msgs::msg::Path path;
  const bool ok = planner_.plan(odom_->pose.pose.position.x, odom_->pose.pose.position.y,
    goal_.point.x, goal_.point.y, *map_, path);
  if (ok) {
    // The map timestamp identifies the latest fused observation; the path
    // timestamp identifies this fresh motion-intent heartbeat.
    path.header.stamp = toMessageTime(get_clock()->now());
    active_path_ = path;
    have_active_path_ = true;
    path_pub_->publish(active_path_);
  } else publishEmptyPath();
  dirty_ = false;
}

void PlannerNode::timerCallback()
{
  if (!goal_active_ || !odom_) return;
  if (planner_.goalReached(odom_->pose.pose.position.x, odom_->pose.pose.position.y,
      goal_.point.x, goal_.point.y)) {
    goal_active_ = false;
    publishEmptyPath();
    RCLCPP_INFO(get_logger(), "Goal reached");
    return;
  }
  planIfReady();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
