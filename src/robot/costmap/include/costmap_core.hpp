#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void configure(double resolution, int width, int height, double origin_x, double origin_y);
    nav_msgs::msg::OccupancyGrid build(const sensor_msgs::msg::LaserScan& scan) const;

  private:
    void markRay(std::vector<int8_t>& data, int x0, int y0, int x1, int y1) const;
    bool toCell(double x, double y, int& ix, int& iy) const;

    rclcpp::Logger logger_;
    double resolution_{0.1};
    int width_{200};
    int height_{200};
    double origin_x_{-10.0};
    double origin_y_{-10.0};

};

}  

#endif
