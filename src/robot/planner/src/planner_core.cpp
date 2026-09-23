#include <algorithm>
#include <cmath>
#include <limits>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

void PlannerCore::configure(double inflation_radius, double unknown_penalty, double goal_tolerance)
{
  inflation_radius_ = inflation_radius;
  unknown_penalty_ = unknown_penalty;
  goal_tolerance_ = goal_tolerance;
}

bool PlannerCore::toCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y, Cell& cell) const
{
  if (!(map.info.resolution > 0.0f) || map.info.width == 0 || map.info.height == 0) return false;
  cell.x = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  cell.y = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  return cell.x >= 0 && cell.x < static_cast<int>(map.info.width) &&
    cell.y >= 0 && cell.y < static_cast<int>(map.info.height);
}

void PlannerCore::toWorld(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell, double& x, double& y) const
{
  x = map.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * map.info.resolution;
  y = map.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * map.info.resolution;
}

double PlannerCore::cellCost(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell) const
{
  if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(map.info.width) ||
    cell.y >= static_cast<int>(map.info.height)) return std::numeric_limits<double>::infinity();
  const auto value = map.data[static_cast<std::size_t>(cell.y * map.info.width + cell.x)];
  if (value < 0) return unknown_penalty_;
  if (value >= 100) return std::numeric_limits<double>::infinity();
  return static_cast<double>(value) / 100.0;
}

double PlannerCore::heuristic(const Cell& a, const Cell& b) const
{
  const double dx = std::abs(a.x - b.x);
  const double dy = std::abs(a.y - b.y);
  return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

bool PlannerCore::goalReached(double x, double y, double goal_x, double goal_y) const
{
  return std::hypot(goal_x - x, goal_y - y) <= goal_tolerance_;
}

bool PlannerCore::plan(double start_x, double start_y, double goal_x, double goal_y,
  const nav_msgs::msg::OccupancyGrid& map, nav_msgs::msg::Path& output) const
{
  output = nav_msgs::msg::Path();
  output.header = map.header;
  const auto& origin_q = map.info.origin.orientation;
  const double origin_yaw = std::atan2(2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
    1.0 - 2.0 * (origin_q.y * origin_q.y + origin_q.z * origin_q.z));
  if (std::abs(origin_yaw) > 1e-3) return false;
  if (map.data.size() != static_cast<std::size_t>(map.info.width) * map.info.height) return false;
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int radius = static_cast<int>(std::ceil(inflation_radius_ / map.info.resolution));
  std::vector<std::uint8_t> blocked(map.data.size(), 0U);
  // A candidate whose enclosing clearance reaches beyond the published map is
  // invalid even when the map has no obstacle at its center.
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (x < radius || y < radius || x + radius >= width || y + radius >= height) {
        blocked[static_cast<std::size_t>(y * width + x)] = 1U;
      }
    }
  }
  for (int obstacle_y = 0; obstacle_y < height; ++obstacle_y) {
    for (int obstacle_x = 0; obstacle_x < width; ++obstacle_x) {
      const auto obstacle_index = static_cast<std::size_t>(obstacle_y * width + obstacle_x);
      if (map.data[obstacle_index] < 100) continue;
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
          const double distance = std::hypot(dx * map.info.resolution, dy * map.info.resolution);
          if (distance > inflation_radius_ + map.info.resolution * 0.71) continue;
          const int x = obstacle_x + dx;
          const int y = obstacle_y + dy;
          if (x >= 0 && y >= 0 && x < width && y < height) {
            blocked[static_cast<std::size_t>(y * width + x)] = 1U;
          }
        }
      }
    }
  }
  const auto isBlocked = [&blocked, width, height](const Cell& cell) {
      return cell.x < 0 || cell.y < 0 || cell.x >= width || cell.y >= height ||
        blocked[static_cast<std::size_t>(cell.y * width + cell.x)] != 0U;
    };
  Cell start;
  Cell goal;
  if (!toCell(map, start_x, start_y, start) || !toCell(map, goal_x, goal_y, goal)) return false;
  if (isBlocked(start) || isBlocked(goal)) return false;

  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
  std::unordered_map<Cell, double, CellHash> g_score;
  std::unordered_map<Cell, Cell, CellHash> parent;
  g_score[start] = 0.0;
  open.push(QueueItem{start, heuristic(start, goal)});

  static constexpr int offsets[8][2] = {{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1}};
  bool found = false;
  while (!open.empty()) {
    const auto current = open.top().cell;
    const double current_g = g_score[current];
    open.pop();
    if (current == goal) { found = true; break; }
    for (const auto& offset : offsets) {
      Cell next{current.x + offset[0], current.y + offset[1]};
      if (isBlocked(next)) continue;
      if (offset[0] != 0 && offset[1] != 0 &&
          (isBlocked(Cell{next.x, current.y}) || isBlocked(Cell{current.x, next.y}))) continue;
      const double step = (offset[0] != 0 && offset[1] != 0) ? std::sqrt(2.0) : 1.0;
      const double penalty = cellCost(map, next);
      if (!std::isfinite(penalty)) continue;
      const double tentative = current_g + step * (1.0 + penalty);
      const auto found_g = g_score.find(next);
      if (found_g == g_score.end() || tentative < found_g->second) {
        g_score[next] = tentative;
        parent[next] = current;
        open.push(QueueItem{next, tentative + heuristic(next, goal)});
      }
    }
  }
  if (!found) return false;

  std::vector<Cell> cells;
  for (Cell current = goal;;) {
    cells.push_back(current);
    if (current == start) break;
    const auto it = parent.find(current);
    if (it == parent.end()) return false;
    current = it->second;
  }
  std::reverse(cells.begin(), cells.end());

  output.poses.reserve(cells.size() + 2);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = map.header;
    double x = 0.0;
    double y = 0.0;
    toWorld(map, cells[i], x, y);
    if (i == 0) { x = start_x; y = start_y; }
    if (i + 1 == cells.size()) { x = goal_x; y = goal_y; }
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    const std::size_t next_index = std::min(i + 1, cells.size() - 1);
    double nx = 0.0;
    double ny = 0.0;
    toWorld(map, cells[next_index], nx, ny);
    if (next_index == i) { nx = goal_x; ny = goal_y; }
    pose.pose.orientation.z = std::sin(0.5 * std::atan2(ny - y, nx - x));
    pose.pose.orientation.w = std::cos(0.5 * std::atan2(ny - y, nx - x));
    output.poses.push_back(pose);
  }
  return !output.poses.empty();
}

} 
