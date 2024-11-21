/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
#define OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <utility>
#include <vector>

namespace occupancy_maze_simulator
{

struct Obstacle
{
  double x;
  double y;
  double width;
  double height;
  double angle;
};

class OccupancyMazeSimulator : public rclcpp::Node
{
public:
  explicit OccupancyMazeSimulator(const rclcpp::NodeOptions & options);

private:
  nav_msgs::msg::OccupancyGrid create_grid_map(
    const std::vector<Obstacle> & obstacles, const std::pair<int, int> & area_size,
    float cell_size);

  nav_msgs::msg::OccupancyGrid create_empty_grid_map();

  void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);

  static std::vector<Obstacle> generate_random_obstacles(
    int num_obstacles, const std::pair<int, int> & area_size);

  std::vector<Obstacle> generate_maze_obstacles(
    float cell_size, const std::pair<int, int> & area_size) const;

  static Obstacle create_obstacle(double x, double y, double width, double height, double angle);

  static bool is_path_to_goal(
    const nav_msgs::msg::OccupancyGrid & grid_map, const std::pair<int, int> & start,
    const std::pair<int, int> & goal);

  // Simple option to simulate robot movement (default)
  void simulate_robot_position(geometry_msgs::msg::Twist::SharedPtr msg);
  // Alternative option to simulate robot movement (currently not implemented)
  void simulate_drone_movement(geometry_msgs::msg::Twist::SharedPtr target_twist);

  void publish_pose();

  void publish_gridmap();

  void publish_slam_gridmap();

  void simulate_lidar_scan();

  rclcpp::TimerBase::SharedPtr publish_pose_timer_;
  rclcpp::TimerBase::SharedPtr publish_gridmap_timer_;
  rclcpp::TimerBase::SharedPtr publish_slam_gridmap_timer_;
  rclcpp::Time last_update_time_;

  double yaw_ = 0.0F;
  double robot_x_ = 0.0F;
  double robot_y_ = 0.0F;
  double current_linear_velocity_ = 0.0F;
  double current_angular_velocity_ = 0.0F;

  int num_cells_x_;
  int num_cells_y_;
  std::pair<int, int> start_position_;
  std::pair<int, int> goal_position_;
  float cell_size_;
  float maze_density_;
  nav_msgs::msg::OccupancyGrid grid_map_;
  nav_msgs::msg::OccupancyGrid slam_grid_map_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_grid_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
};

}  // namespace occupancy_maze_simulator

#endif  // OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
