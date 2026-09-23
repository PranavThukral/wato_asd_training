#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <memory>
#include <cstdint>
#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void configure(double resolution, int width, int height, double origin_x, double origin_y);
    void setFrame(const std::string& frame_id);
    bool update(const nav_msgs::msg::OccupancyGrid& local, const geometry_msgs::msg::Pose& pose,
      const rclcpp::Time& stamp);
    const nav_msgs::msg::OccupancyGrid& map() const { return global_map_; }
    bool initialized() const { return initialized_; }

  private:
    bool toCell(double x, double y, int& x_cell, int& y_cell) const;
    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool initialized_{false};
    bool configured_{false};
};

}  

#endif  
