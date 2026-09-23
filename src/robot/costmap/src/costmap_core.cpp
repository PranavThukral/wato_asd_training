#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height, double origin_x, double origin_y)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

bool CostmapCore::toCell(double x, double y, int& ix, int& iy) const
{
  ix = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  iy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return ix >= 0 && ix < width_ && iy >= 0 && iy < height_;
}

void CostmapCore::markRay(std::vector<int8_t>& data, int x0, int y0, int x1, int y1) const
{
  int dx = std::abs(x1 - x0);
  int sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0);
  int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
      const auto index = static_cast<std::size_t>(y * width_ + x);
      if (data[index] != 100) data[index] = 0;
    }
    if (x == x1 && y == y1) break;
    const int twice_error = 2 * error;
    if (twice_error >= dy) { error += dy; x += sx; }
    if (twice_error <= dx) { error += dx; y += sy; }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::build(const sensor_msgs::msg::LaserScan& scan) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = scan.header;
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<std::uint32_t>(width_);
  grid.info.height = static_cast<std::uint32_t>(height_);
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width_ * height_), -1);

  int sensor_x = 0;
  int sensor_y = 0;
  if (!toCell(0.0, 0.0, sensor_x, sensor_y)) {
    RCLCPP_ERROR(logger_, "Local costmap origin does not contain the sensor");
    return grid;
  }

  double angle = scan.angle_min;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment) {
    const float raw_range = scan.ranges[i];
    if (std::isnan(raw_range) || raw_range < scan.range_min ||
        (std::isinf(raw_range) && raw_range < 0.0f)) {
      continue;
    }

    // A reading exactly at range_max is treated as a clear-to-range return;
    // Gazebo uses that value for beams that do not hit an obstacle.
    const bool has_hit = std::isfinite(raw_range) && raw_range < scan.range_max &&
      std::isfinite(scan.range_max);
    const double range = std::isinf(raw_range) ? scan.range_max :
      std::min<double>(raw_range, scan.range_max);
    if (!std::isfinite(range) || range <= 0.0) continue;

    const double end_x = range * std::cos(angle);
    const double end_y = range * std::sin(angle);
    int end_cell_x = 0;
    int end_cell_y = 0;
    if (!toCell(end_x, end_y, end_cell_x, end_cell_y)) {
      // A ray clipped by the local map boundary is not evidence of an obstacle.
      continue;
    }
    markRay(grid.data, sensor_x, sensor_y, end_cell_x, end_cell_y);
    if (has_hit) {
      grid.data[static_cast<std::size_t>(end_cell_y * width_ + end_cell_x)] = 100;
    }
  }
  return grid;
}

}
