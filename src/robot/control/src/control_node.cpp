#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "control_node.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double poseYaw(const geometry_msgs::msg::Pose& pose)
{
  const auto& q = pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

struct Pose2d
{
  double x;
  double y;
  double yaw;
};

Pose2d integrate(const Pose2d& start, double linear, double angular, double duration)
{
  Pose2d result = start;
  if (std::abs(angular) < 1e-6) {
    result.x += linear * duration * std::cos(start.yaw);
    result.y += linear * duration * std::sin(start.yaw);
  } else {
    const double radius = linear / angular;
    result.x += radius * (std::sin(start.yaw + angular * duration) - std::sin(start.yaw));
    result.y -= radius * (std::cos(start.yaw + angular * duration) - std::cos(start.yaw));
  }
  result.yaw = normalizeAngle(start.yaw + angular * duration);
  return result;
}

}  // namespace

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger()))
{
  const auto lookahead = declare_parameter<double>("lookahead_distance", 0.50);
  const auto goal_tolerance = declare_parameter<double>("goal_tolerance", 0.20);
  const auto max_linear = declare_parameter<double>("max_linear_speed", 0.20);
  const auto max_angular = declare_parameter<double>("max_angular_speed", 0.60);
  const auto emergency_distance = declare_parameter<double>("emergency_distance", 0.35);
  input_timeout_ = declare_parameter<double>("input_timeout", 0.75);
  path_timeout_ = declare_parameter<double>("path_timeout", 1.00);
  map_timeout_ = declare_parameter<double>("map_timeout", 0.75);
  footprint_radius_ = declare_parameter<double>("footprint_radius", 1.90);
  collision_horizon_ = declare_parameter<double>("collision_horizon", 0.80);
  collision_margin_ = declare_parameter<double>("collision_margin", 0.10);
  control_delay_ = declare_parameter<double>("control_delay", 0.15);
  braking_deceleration_ = declare_parameter<double>("braking_deceleration", 0.50);
  collision_step_ = declare_parameter<double>("collision_step", 0.025);
  linear_acceleration_limit_ = declare_parameter<double>("linear_acceleration_limit", 0.40);
  angular_acceleration_limit_ = declare_parameter<double>("angular_acceleration_limit", 1.50);
  if (!(lookahead > 0.0) || !(goal_tolerance > 0.0) || !(max_linear > 0.0) ||
      !(max_angular > 0.0) || !(emergency_distance > 0.0) || !(input_timeout_ > 0.0) ||
      !(path_timeout_ > 0.0) || !(map_timeout_ > 0.0) || !(footprint_radius_ > 0.0) ||
      !(collision_horizon_ > 0.0) || !(collision_margin_ >= 0.0) || !(control_delay_ >= 0.0) ||
      !(braking_deceleration_ > 0.0) || !(collision_step_ > 0.0) ||
      !(linear_acceleration_limit_ > 0.0) || !(angular_acceleration_limit_ > 0.0)) {
    throw std::invalid_argument("Invalid controller parameters");
  }
  control_.configure(lookahead, goal_tolerance, max_linear, max_angular);
  emergency_distance_ = emergency_distance;
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(), std::bind(&ControlNode::scanCallback, this, std::placeholders::_1));
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&ControlNode::mapCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&ControlNode::controlLoop, this));
  publishStop();
}

void ControlNode::pathCallback(nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg) {
    control_.setPath(*msg);
    path_frame_id_ = msg->header.frame_id;
    last_path_stamp_ = rclcpp::Time(msg->header.stamp);
    last_path_receive_ = std::chrono::steady_clock::now();
  }
}

void ControlNode::odomCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg && std::isfinite(msg->pose.pose.position.x) && std::isfinite(msg->pose.pose.position.y) &&
      std::isfinite(msg->pose.pose.orientation.x) && std::isfinite(msg->pose.pose.orientation.y) &&
      std::isfinite(msg->pose.pose.orientation.z) && std::isfinite(msg->pose.pose.orientation.w) &&
      msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
      msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
      msg->pose.pose.orientation.w * msg->pose.pose.orientation.w > 1e-8) {
    odom_ = std::move(msg);
    last_odom_stamp_ = rclcpp::Time(odom_->header.stamp);
    last_odom_receive_ = std::chrono::steady_clock::now();
  }
}

void ControlNode::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg || msg->ranges.empty() || !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_max) ||
      !std::isfinite(msg->angle_increment) || !std::isfinite(msg->range_min) ||
      !std::isfinite(msg->range_max) || msg->angle_increment == 0.0 ||
      !(msg->range_max > msg->range_min)) return;
  bool usable = false;
  for (const double range : msg->ranges) {
    if ((std::isinf(range) && range > 0.0) ||
        (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)) {
      usable = true;
      break;
    }
  }
  // An all-NaN scan must not refresh the safety watchdog.
  if (!usable) return;
  scan_ = std::move(msg);
  last_scan_stamp_ = rclcpp::Time(scan_->header.stamp);
  last_scan_receive_ = std::chrono::steady_clock::now();
}

void ControlNode::mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->info.width == 0 || msg->info.height == 0 || msg->data.empty()) return;
  if (msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height) return;
  map_ = std::move(msg);
  last_map_stamp_ = rclcpp::Time(map_->header.stamp);
  last_map_receive_ = std::chrono::steady_clock::now();
}

void ControlNode::publishStop()
{
  previous_command_ = geometry_msgs::msg::Twist();
  previous_command_time_ = std::chrono::steady_clock::now();
  cmd_pub_->publish(previous_command_);
}

bool ControlNode::inputsFresh(const rclcpp::Time& now) const
{
  const auto steady_now = std::chrono::steady_clock::now();
  const auto freshSteady = [steady_now](const std::chrono::steady_clock::time_point& received,
      double timeout) {
      if (received.time_since_epoch().count() == 0) return false;
      return std::chrono::duration<double>(steady_now - received).count() <= timeout;
    };
  if (!freshSteady(last_odom_receive_, input_timeout_) ||
      !freshSteady(last_scan_receive_, input_timeout_) ||
      !freshSteady(last_map_receive_, map_timeout_) ||
      !freshSteady(last_path_receive_, path_timeout_)) return false;

  // A map is only navigation-ready after a real scan has been fused. Its initial
  // diagnostic publication deliberately has a zero timestamp.
  if (last_map_stamp_.nanoseconds() == 0) return false;
  if (now.nanoseconds() <= 0) return true;
  const auto sourceAgeOkay = [&now](const rclcpp::Time& stamp, double timeout, bool required) {
      if (required && stamp.nanoseconds() == 0) return false;
      if (stamp.nanoseconds() == 0) return true;
      const double age = (now - stamp).seconds();
      return age >= -0.25 && age <= timeout;
    };
  return sourceAgeOkay(last_odom_stamp_, input_timeout_, false) &&
    sourceAgeOkay(last_scan_stamp_, input_timeout_, false) &&
    sourceAgeOkay(last_map_stamp_, map_timeout_, true) &&
    sourceAgeOkay(last_path_stamp_, path_timeout_, false);
}

bool ControlNode::scanCovers(double x, double y) const
{
  if (!scan_ || !odom_ || scan_->ranges.empty() || scan_->angle_increment == 0.0) return false;
  const double dx = x - odom_->pose.pose.position.x;
  const double dy = y - odom_->pose.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double angle = normalizeAngle(std::atan2(dy, dx) -
    std::atan2(2.0 * (odom_->pose.pose.orientation.w * odom_->pose.pose.orientation.z +
      odom_->pose.pose.orientation.x * odom_->pose.pose.orientation.y),
      1.0 - 2.0 * (odom_->pose.pose.orientation.y * odom_->pose.pose.orientation.y +
        odom_->pose.pose.orientation.z * odom_->pose.pose.orientation.z)));
  const double increment = scan_->angle_increment;
  const double raw_index = (angle - scan_->angle_min) / increment;
  const auto index = static_cast<long>(std::llround(raw_index));
  if (index < 0 || index >= static_cast<long>(scan_->ranges.size())) return false;
  const double beam_angle = scan_->angle_min + static_cast<double>(index) * increment;
  if (std::abs(normalizeAngle(angle - beam_angle)) > std::max(0.04, std::abs(increment) * 0.75)) return false;
  const double range = scan_->ranges[static_cast<std::size_t>(index)];
  if (std::isinf(range) && range > 0.0) return distance <= scan_->range_max;
  if (!std::isfinite(range) || range < scan_->range_min) return false;
  return distance <= range + collision_margin_;
}

bool ControlNode::footprintClear(double x, double y, bool allow_unknown) const
{
  if (!map_ || map_->info.resolution <= 0.0f || map_->data.empty()) return false;
  const auto& origin = map_->info.origin;
  const auto& q = origin.orientation;
  const double origin_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (std::abs(origin_yaw) > 1e-3) return false;
  const double radius = footprint_radius_ + collision_margin_;
  const double resolution = map_->info.resolution;
  const int min_x = static_cast<int>(std::floor((x - radius - origin.position.x) / resolution));
  const int max_x = static_cast<int>(std::floor((x + radius - origin.position.x) / resolution));
  const int min_y = static_cast<int>(std::floor((y - radius - origin.position.y) / resolution));
  const int max_y = static_cast<int>(std::floor((y + radius - origin.position.y) / resolution));
  for (int gy = min_y; gy <= max_y; ++gy) {
    for (int gx = min_x; gx <= max_x; ++gx) {
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_->info.width) ||
          gy >= static_cast<int>(map_->info.height)) return false;
      const double cell_x = origin.position.x + (static_cast<double>(gx) + 0.5) * resolution;
      const double cell_y = origin.position.y + (static_cast<double>(gy) + 0.5) * resolution;
      if (std::hypot(cell_x - x, cell_y - y) > radius) continue;
      const auto value = map_->data[static_cast<std::size_t>(gy) * map_->info.width + gx];
      if (value >= 100) return false;
      if (value < 0 && !allow_unknown) {
        // The odometry pose is the lidar frame, so the chassis itself can
        // legitimately occupy cells that the lidar cannot observe behind it.
        const bool current_footprint = odom_ &&
          std::hypot(cell_x - odom_->pose.pose.position.x, cell_y - odom_->pose.pose.position.y) <=
          footprint_radius_ + resolution;
        if (!current_footprint && !scanCovers(cell_x, cell_y)) return false;
      }
    }
  }
  return true;
}

bool ControlNode::scanArcClear(const geometry_msgs::msg::Pose& pose,
  const geometry_msgs::msg::Twist& command, bool allow_current_occupied) const
{
  if (!scan_) return false;
  if (command.linear.x > 1e-6) {
    double nearest_front = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
      const double range = scan_->ranges[i];
      if (std::abs(angle) <= 0.65 && std::isfinite(range) && range >= scan_->range_min && range < scan_->range_max) {
        nearest_front = std::min(nearest_front, range);
      }
    }
    const double stopping_distance = command.linear.x * control_delay_ +
      command.linear.x * command.linear.x / (2.0 * braking_deceleration_);
    if (nearest_front < footprint_radius_ + collision_margin_ + stopping_distance) return false;
  }

  const double start_yaw = std::atan2(2.0 * (pose.orientation.w * pose.orientation.z +
      pose.orientation.x * pose.orientation.y),
    1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z));
  const Pose2d start{pose.position.x, pose.position.y, start_yaw};
  const double horizon = std::max(collision_horizon_, control_delay_ + 0.1);
  const int samples = std::max(1, static_cast<int>(std::ceil(horizon / collision_step_)));
  for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
    const double range = scan_->ranges[i];
    if (!std::isfinite(range) || range < scan_->range_min || range >= scan_->range_max) continue;
    const double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
    const double obstacle_x = start.x + range * std::cos(start.yaw + angle);
    const double obstacle_y = start.y + range * std::sin(start.yaw + angle);
    for (int sample = 0; sample <= samples; ++sample) {
      if (allow_current_occupied && sample == 0) continue;
      const double t = horizon * static_cast<double>(sample) / samples;
      const auto predicted = integrate(start, command.linear.x, command.angular.z, t);
      if (std::hypot(obstacle_x - predicted.x, obstacle_y - predicted.y) <=
          footprint_radius_ + collision_margin_) return false;
    }
  }
  return true;
}

bool ControlNode::trajectoryClear(const geometry_msgs::msg::Twist& command,
  bool allow_current_occupied) const
{
  if (!odom_ || !map_ || !scan_) return false;
  const double start_yaw = std::atan2(2.0 * (odom_->pose.pose.orientation.w * odom_->pose.pose.orientation.z +
      odom_->pose.pose.orientation.x * odom_->pose.pose.orientation.y),
    1.0 - 2.0 * (odom_->pose.pose.orientation.y * odom_->pose.pose.orientation.y +
      odom_->pose.pose.orientation.z * odom_->pose.pose.orientation.z));
  const Pose2d start{odom_->pose.pose.position.x, odom_->pose.pose.position.y, start_yaw};
  const double horizon = std::max(collision_horizon_, control_delay_ + 0.1);
  const int samples = std::max(1, static_cast<int>(std::ceil(horizon / collision_step_)));
  if (!allow_current_occupied) {
    for (int sample = 0; sample <= samples; ++sample) {
      const double t = horizon * static_cast<double>(sample) / samples;
      const auto predicted = integrate(start, command.linear.x, command.angular.z, t);
      if (!footprintClear(predicted.x, predicted.y, sample == 0)) return false;
    }
  }
  return scanArcClear(odom_->pose.pose, command, allow_current_occupied);
}

geometry_msgs::msg::Twist ControlNode::recoveryCommand() const
{
  geometry_msgs::msg::Twist command;
  command.linear.x = -0.25;
  double left_clearance = std::numeric_limits<double>::infinity();
  double right_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
    const double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
    const double range = scan_->ranges[i];
    if (!std::isfinite(range) || range < scan_->range_min || range >= scan_->range_max) continue;
    if (angle > 0.25 && angle < 1.40) left_clearance = std::min(left_clearance, range);
    if (angle < -0.25 && angle > -1.40) right_clearance = std::min(right_clearance, range);
  }
  if (left_clearance > right_clearance) {
    command.angular.z = 0.80;
  } else {
    command.angular.z = -0.80;
  }
  return command;
}

bool ControlNode::publishRecovery()
{
  if (!scan_) return false;
  const auto recovery = recoveryCommand();
  if (!trajectoryClear(recovery, true)) return false;
  previous_command_ = recovery;
  previous_command_time_ = std::chrono::steady_clock::now();
  cmd_pub_->publish(recovery);
  return true;
}

void ControlNode::controlLoop()
{
  if (!odom_ || !scan_ || !map_) { publishStop(); return; }
  const rclcpp::Time now = get_clock()->now();
  if (!inputsFresh(now)) { publishStop(); return; }
  if (map_->header.frame_id.empty() ||
      (!path_frame_id_.empty() && path_frame_id_ != map_->header.frame_id) ||
      (!odom_->header.frame_id.empty() && odom_->header.frame_id != map_->header.frame_id)) {
    publishStop();
    return;
  }
  if (!control_.validPath()) {
    if (!footprintClear(odom_->pose.pose.position.x, odom_->pose.pose.position.y, true)) {
      if (!publishRecovery()) publishStop();
    } else {
      publishStop();
    }
    return;
  }
  bool emergency = false;
  for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
    const double angle = scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
    const double range = scan_->ranges[i];
    if (std::isfinite(range) && range >= scan_->range_min && range < emergency_distance_ && std::abs(angle) < 0.65) {
      emergency = true;
      break;
    }
  }
  if (emergency) {
    ++blocked_cycles_;
    if (blocked_cycles_ < 3 || !publishRecovery()) publishStop();
    return;
  }
  auto command = control_.command(odom_->pose.pose);
  if (!std::isfinite(command.linear.x) || !std::isfinite(command.angular.z)) {
    publishStop();
    return;
  }
  const auto steady_now = std::chrono::steady_clock::now();
  double dt = 0.05;
  if (previous_command_time_.time_since_epoch().count() != 0) {
    dt = std::clamp(std::chrono::duration<double>(steady_now - previous_command_time_).count(), 0.001, 0.25);
  }
  command.linear.x = std::clamp(command.linear.x,
    previous_command_.linear.x - linear_acceleration_limit_ * dt,
    previous_command_.linear.x + linear_acceleration_limit_ * dt);
  command.angular.z = std::clamp(command.angular.z,
    previous_command_.angular.z - angular_acceleration_limit_ * dt,
    previous_command_.angular.z + angular_acceleration_limit_ * dt);
  if (std::abs(command.linear.x) < 1e-6 && std::abs(command.angular.z) < 1e-6) {
    publishStop();
    return;
  }
  if (!trajectoryClear(command)) {
    ++blocked_cycles_;
    if (blocked_cycles_ < 3) {
      publishStop();
      return;
    }
    command = recoveryCommand();
    if (!trajectoryClear(command, true)) {
      publishStop();
      return;
    }
    ++recovery_cycles_;
    if (recovery_cycles_ > 20) {
      blocked_cycles_ = 0;
      recovery_cycles_ = 0;
      publishStop();
      return;
    }
  } else {
    blocked_cycles_ = 0;
    recovery_cycles_ = 0;
  }
  previous_command_ = command;
  previous_command_time_ = steady_now;
  cmd_pub_->publish(command);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
