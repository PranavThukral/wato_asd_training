#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void mapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(geometry_msgs::msg::PointStamped::SharedPtr msg);
    void poseGoalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishEmptyPath();
    void planIfReady();

    robot::PlannerCore planner_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    nav_msgs::msg::Path active_path_;
    geometry_msgs::msg::PointStamped goal_;
    double map_timeout_{0.75};
    rclcpp::Time last_map_stamp_{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point last_map_receive_{};
    bool goal_active_{false};
    bool dirty_{false};
    bool have_active_path_{false};
};

#endif 
