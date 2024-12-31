/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
#define OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/save_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <limits>
#include <memory>
#include <string>
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
  nav_msgs::msg::OccupancyGrid create_grid_map(const std::vector<Obstacle> & obstacles);

  nav_msgs::msg::OccupancyGrid create_empty_grid_map();

  void twist_callback(geometry_msgs::msg::Twist::SharedPtr msg);

  std::vector<Obstacle> generate_random_obstacles(int num_obstacles) const;

  std::vector<Obstacle> generate_maze_obstacles() const;

  static Obstacle create_obstacle(double x, double y, double width, double height, double angle);

  bool is_path_to_target(
    const nav_msgs::msg::OccupancyGrid & grid_map, geometry_msgs::msg::PoseStamped & start,
    geometry_msgs::msg::PoseStamped & target) const;

  // Simple option to simulate robot movement (default)
  void simulate_robot_position(geometry_msgs::msg::Twist::SharedPtr msg);
  // Alternative option to simulate robot movement (currently not implemented)
  // void simulate_drone_movement(geometry_msgs::msg::Twist::SharedPtr target_twist);

  void publish_pose();

  void publish_gridmap();

  void publish_slam_gridmap();

  void simulate_lidar_scan();

  void reset_callback(std_msgs::msg::Empty::SharedPtr msg);

  void handle_load_map_response(rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture result);

  std_msgs::msg::ColorRGBA default_color_;
  void publish_text_marker(
    std::string visualize_text, geometry_msgs::msg::Pose marker_pose,
    std_msgs::msg::ColorRGBA text_color);

  void record_statistics(std::string failed_msg = "");

  void failed_callback(std_msgs::msg::String::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr publish_pose_timer_;
  rclcpp::TimerBase::SharedPtr publish_gridmap_timer_;
  rclcpp::TimerBase::SharedPtr publish_slam_gridmap_timer_;
  rclcpp::Time last_update_time_;

  geometry_msgs::msg::PoseStamped start_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;

  double yaw_ = 0.0;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double current_linear_velocity_ = 0.0;
  double current_angular_velocity_ = 0.0;

  int width_;
  int height_;

  float resolution_;

  double gridmap_origin_x_;
  double gridmap_origin_y_;
  float maze_density_;
  std::string obstacle_mode_;
  nav_msgs::msg::OccupancyGrid grid_map_;
  nav_msgs::msg::OccupancyGrid slam_grid_map_;
  std::string robot_pose_topic_;
  std::string robot_velocity_topic_;
  int robot_count_ = 1;

  // 自動評価のための変数
  std::string csv_stat_file_name_;
  std::vector<double> travel_times_;
  std::vector<double> travel_speeds_;
  double max_speed_ = 0.0;
  double min_speed_ = std::numeric_limits<double>::max();
  double min_distance_to_object_ = std::numeric_limits<double>::max();
  int trial_count_ = 0;
  rclcpp::Time start_time_;
  bool is_reached_to_target_ = false;
  int record_count_ = 0;
  int max_trial_count_;
  double timeout_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slam_grid_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr text_marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_stop_publisher_;
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr failed_subscriber_;
};

}  // namespace occupancy_maze_simulator

#endif  // OCCUPANCY_MAZE_SIMULATOR__OCCUPANCY_MAZE_SIMULATOR_HPP_
