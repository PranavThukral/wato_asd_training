#include <algorithm>
#include <cmath>

#include "control_core.hpp"

namespace robot
{

namespace { constexpr double kPi = 3.14159265358979323846; }

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void ControlCore::configure(double lookahead_distance, double goal_tolerance,
  double max_linear_speed, double max_angular_speed)
{
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  max_linear_speed_ = max_linear_speed;
  max_angular_speed_ = max_angular_speed;
}

void ControlCore::setPath(const nav_msgs::msg::Path& path)
{
  path_ = path;
}

bool ControlCore::validPath() const
{
  if (path_.poses.empty()) return false;
  for (const auto& waypoint : path_.poses) {
    if (!std::isfinite(waypoint.pose.position.x) || !std::isfinite(waypoint.pose.position.y) ||
        !std::isfinite(waypoint.pose.orientation.x) || !std::isfinite(waypoint.pose.orientation.y) ||
        !std::isfinite(waypoint.pose.orientation.z) || !std::isfinite(waypoint.pose.orientation.w)) {
      return false;
    }
  }
  return true;
}

double ControlCore::yaw(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double ControlCore::normalizeAngle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

bool ControlCore::goalReached(const geometry_msgs::msg::Pose& pose) const
{
  if (path_.poses.empty()) return true;
  const auto& goal = path_.poses.back().pose.position;
  return std::hypot(goal.x - pose.position.x, goal.y - pose.position.y) <= goal_tolerance_;
}

bool ControlCore::findLookahead(const geometry_msgs::msg::Pose& pose, double& x, double& y) const
{
  if (path_.poses.empty()) return false;
  double accumulated = 0.0;
  double previous_x = pose.position.x;
  double previous_y = pose.position.y;
  for (const auto& waypoint : path_.poses) {
    const double wx = waypoint.pose.position.x;
    const double wy = waypoint.pose.position.y;
    const double segment = std::hypot(wx - previous_x, wy - previous_y);
    if (accumulated + segment >= lookahead_distance_ && segment > 1e-6) {
      const double fraction = std::clamp((lookahead_distance_ - accumulated) / segment, 0.0, 1.0);
      x = previous_x + fraction * (wx - previous_x);
      y = previous_y + fraction * (wy - previous_y);
      return true;
    }
    accumulated += segment;
    previous_x = wx;
    previous_y = wy;
  }
  x = path_.poses.back().pose.position.x;
  y = path_.poses.back().pose.position.y;
  return true;
}

geometry_msgs::msg::Twist ControlCore::command(const geometry_msgs::msg::Pose& pose) const
{
  geometry_msgs::msg::Twist cmd;
  if (!validPath() || goalReached(pose)) return cmd;
  double target_x = 0.0;
  double target_y = 0.0;
  if (!findLookahead(pose, target_x, target_y)) return cmd;

  const double dx = target_x - pose.position.x;
  const double dy = target_y - pose.position.y;
  const double distance_sq = dx * dx + dy * dy;
  if (distance_sq < 1e-8) return cmd;
  const double heading = yaw(pose.orientation);
  const double target_heading = std::atan2(dy, dx);
  const double heading_error = normalizeAngle(target_heading - heading);
  if (std::abs(heading_error) > kPi / 3.0) {
    cmd.angular.z = std::clamp(heading_error, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }
  const double local_y = -std::sin(heading) * dx + std::cos(heading) * dy;
  const double curvature = 2.0 * local_y / distance_sq;
  double speed = max_linear_speed_;
  if (std::abs(curvature) > 1e-6) speed = std::min(speed, max_angular_speed_ / std::abs(curvature));
  const double goal_distance = std::hypot(path_.poses.back().pose.position.x - pose.position.x,
    path_.poses.back().pose.position.y - pose.position.y);
  speed = std::min(speed, std::max(0.03, goal_distance));
  cmd.linear.x = std::max(0.0, speed);
  cmd.angular.z = std::clamp(speed * curvature, -max_angular_speed_, max_angular_speed_);
  return cmd;
}

}  
