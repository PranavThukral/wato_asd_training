#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr msg);
    void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void controlLoop();
    void publishStop();
    bool inputsFresh(const rclcpp::Time& now) const;
    bool trajectoryClear(const geometry_msgs::msg::Twist& command) const;
    bool footprintClear(double x, double y, bool allow_unknown) const;
    bool scanCovers(double x, double y) const;
    bool scanArcClear(const geometry_msgs::msg::Pose& pose,
      const geometry_msgs::msg::Twist& command) const;

    robot::ControlCore control_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::string path_frame_id_;
    double emergency_distance_{0.35};
    double input_timeout_{0.75};
    double path_timeout_{1.0};
    double map_timeout_{0.75};
    double footprint_radius_{1.90};
    double collision_horizon_{0.80};
    double collision_margin_{0.10};
    double control_delay_{0.15};
    double braking_deceleration_{0.50};
    double collision_step_{0.025};
    double linear_acceleration_limit_{0.40};
    double angular_acceleration_limit_{1.50};
    geometry_msgs::msg::Twist previous_command_;
    std::chrono::steady_clock::time_point previous_command_time_{};
    rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_path_stamp_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_odom_receive_{};
    std::chrono::steady_clock::time_point last_scan_receive_{};
    std::chrono::steady_clock::time_point last_map_receive_{};
    std::chrono::steady_clock::time_point last_path_receive_{};
};

#endif
