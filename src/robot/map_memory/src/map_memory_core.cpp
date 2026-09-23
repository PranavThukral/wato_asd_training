#include <algorithm>
#include <cmath>
#include <cstdint>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::configure(double resolution, int width, int height, double origin_x, double origin_y)
{
  global_map_ = nav_msgs::msg::OccupancyGrid();
  global_map_.info.resolution = static_cast<float>(resolution);
  global_map_.info.width = static_cast<std::uint32_t>(width);
  global_map_.info.height = static_cast<std::uint32_t>(height);
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(static_cast<std::size_t>(width * height), -1);
  configured_ = true;
  initialized_ = false;
}

void MapMemoryCore::setFrame(const std::string& frame_id)
{
  if (!frame_id.empty()) global_map_.header.frame_id = frame_id;
}

double MapMemoryCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

bool MapMemoryCore::toCell(double x, double y, int& x_cell, int& y_cell) const
{
  const double resolution = global_map_.info.resolution;
  x_cell = static_cast<int>(std::floor((x - global_map_.info.origin.position.x) / resolution));
  y_cell = static_cast<int>(std::floor((y - global_map_.info.origin.position.y) / resolution));
  return x_cell >= 0 && x_cell < static_cast<int>(global_map_.info.width) &&
    y_cell >= 0 && y_cell < static_cast<int>(global_map_.info.height);
}

bool MapMemoryCore::update(const nav_msgs::msg::OccupancyGrid& local,
  const geometry_msgs::msg::Pose& pose, const rclcpp::Time& stamp)
{
  if (!configured_ || local.info.resolution <= 0.0f || local.data.empty() ||
      local.data.size() != static_cast<std::size_t>(local.info.width) * local.info.height) return false;
  const auto& local_q = local.info.origin.orientation;
  const double local_origin_yaw = std::atan2(2.0 * (local_q.w * local_q.z + local_q.x * local_q.y),
    1.0 - 2.0 * (local_q.y * local_q.y + local_q.z * local_q.z));
  if (std::abs(local_origin_yaw) > 1e-3) return false;
  const double yaw = yawFromQuaternion(pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double local_resolution = local.info.resolution;
  bool observed = false;

  for (std::uint32_t ly = 0; ly < local.info.height; ++ly) {
    for (std::uint32_t lx = 0; lx < local.info.width; ++lx) {
      const auto local_index = static_cast<std::size_t>(ly * local.info.width + lx);
      const int8_t value = local.data[local_index];
      if (value < 0) continue;
      observed = true;
      const double local_x = local.info.origin.position.x + (static_cast<double>(lx) + 0.5) * local_resolution;
      const double local_y = local.info.origin.position.y + (static_cast<double>(ly) + 0.5) * local_resolution;
      const double world_x = pose.position.x + c * local_x - s * local_y;
      const double world_y = pose.position.y + s * local_x + c * local_y;
      int gx = 0;
      int gy = 0;
      if (!toCell(world_x, world_y, gx, gy)) continue;
      auto& destination = global_map_.data[static_cast<std::size_t>(gy * global_map_.info.width + gx)];
      // Occupied evidence is conservative. Free evidence is allowed to clear stale raw evidence.
      if (value >= 100) destination = 100;
      else if (destination != 100 || value == 0) destination = value;
    }
  }
  if (!observed) return false;
  // rclcpp::Time in the supplied ROS 2 image does not expose to_msg().
  // Preserve the exact nanosecond timestamp when assigning the ROS message.
  auto nanoseconds = stamp.nanoseconds();
  auto seconds = nanoseconds / 1000000000LL;
  auto remainder = nanoseconds % 1000000000LL;
  if (remainder < 0) {
    --seconds;
    remainder += 1000000000LL;
  }
  global_map_.header.stamp.sec = static_cast<std::int32_t>(seconds);
  global_map_.header.stamp.nanosec = static_cast<std::uint32_t>(remainder);
  initialized_ = true;
  return true;
}

} 
