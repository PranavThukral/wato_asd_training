#include <memory>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  const auto resolution = declare_parameter<double>("resolution", 0.10);
  const auto width = declare_parameter<int>("width", 200);
  const auto height = declare_parameter<int>("height", 200);
  const auto origin_x = declare_parameter<double>("origin_x", -10.0);
  const auto origin_y = declare_parameter<double>("origin_y", -10.0);
  if (!(resolution > 0.0) || width <= 0 || height <= 0) {
    throw std::invalid_argument("Invalid costmap geometry parameters");
  }
  costmap_.configure(resolution, width, height, origin_x, origin_y);

  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", rclcpp::SensorDataQoS());
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::scanCallback, this, std::placeholders::_1));
}

void CostmapNode::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!scan || !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_max) ||
      !std::isfinite(scan->angle_increment) || !std::isfinite(scan->range_min) ||
      !std::isfinite(scan->range_max) || scan->angle_increment == 0.0 ||
      !(scan->range_max > scan->range_min)) return;
  costmap_pub_->publish(costmap_.build(*scan));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
