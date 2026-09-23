#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    void configure(double inflation_radius, double unknown_penalty, double goal_tolerance);
    bool plan(double start_x, double start_y, double goal_x, double goal_y,
      const nav_msgs::msg::OccupancyGrid& map, nav_msgs::msg::Path& output) const;
    bool goalReached(double x, double y, double goal_x, double goal_y) const;

  private:
    struct Cell {
      int x{0};
      int y{0};
      bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
    };
    struct CellHash {
      std::size_t operator()(const Cell& c) const {
        return std::hash<int>{}(c.x) ^ (std::hash<int>{}(c.y) << 1U);
      }
    };
    struct QueueItem {
      Cell cell;
      double f{0.0};
      bool operator>(const QueueItem& other) const { return f > other.f; }
    };

    bool toCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y, Cell& cell) const;
    void toWorld(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell, double& x, double& y) const;
    double cellCost(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell) const;
    double heuristic(const Cell& a, const Cell& b) const;

    rclcpp::Logger logger_;
    double inflation_radius_{1.95};
    double unknown_penalty_{2.0};
    double goal_tolerance_{0.20};
};

}  

#endif  
