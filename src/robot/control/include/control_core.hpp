#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <cstddef>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double goal_tolerance,
      double max_linear_speed, double max_angular_speed);
    void setPath(const nav_msgs::msg::Path& path);
    bool hasPath() const { return !path_.poses.empty(); }
    bool validPath() const;
    bool goalReached(const geometry_msgs::msg::Pose& pose) const;
    geometry_msgs::msg::Twist command(const geometry_msgs::msg::Pose& pose) const;

  private:
    static double yaw(const geometry_msgs::msg::Quaternion& q);
    static double normalizeAngle(double angle);
    bool findLookahead(const geometry_msgs::msg::Pose& pose, double& x, double& y) const;

    rclcpp::Logger logger_;
    nav_msgs::msg::Path path_;
    double lookahead_distance_{0.50};
    double goal_tolerance_{0.20};
    double max_linear_speed_{0.20};
    double max_angular_speed_{0.60};
};

} 

#endif 
