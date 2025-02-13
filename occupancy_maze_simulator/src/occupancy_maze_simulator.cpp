/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include <occupancy_maze_simulator/occupancy_maze_simulator.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <queue>
#include <random>
#include <string>
#include <vector>

namespace occupancy_maze_simulator
{

OccupancyMazeSimulator::OccupancyMazeSimulator(const rclcpp::NodeOptions & options)
: Node("occupancy_maze_simulator", options),
  tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
{
  this->declare_parameter<std::string>("obstacle_mode", "maze");  // 障害物配置モード:maze / random
  this->declare_parameter<float>("gridmap.resolution", 1);
  this->declare_parameter<float>("gridmap.x", 50);
  this->declare_parameter<float>("gridmap.y", 50);
  this->declare_parameter<float>("gridmap.origin_x", 0.0);
  this->declare_parameter<float>("gridmap.origin_y", 0.0);
  this->declare_parameter<float>("maze.density", 0.3);  // 障害物の密度（0.0～1.0）
  this->declare_parameter<int>("max_trial_count", 100);
  this->declare_parameter<double>("simulation_timeout", 100.0);  // 秒
  this->declare_parameter<std::string>("robot_pose_topic", "mavros/vision_pose/pose");
  this->declare_parameter("robot_velocity_topic", "mavros/setpoint_velocity/cmd_vel_unstamped");
  this->declare_parameter("robot_count", robot_count_);
  this->declare_parameter("start_position.x", -20.0);
  this->declare_parameter("start_position.y", -20.0);
  this->declare_parameter("start_position.z", 0.0);
  this->declare_parameter("start_position.yaw", 0.0);

  this->get_parameter("obstacle_mode", obstacle_mode_);
  this->get_parameter("gridmap.resolution", resolution_);
  float gridmap_x = this->get_parameter("gridmap.x").as_double();
  float gridmap_y = this->get_parameter("gridmap.y").as_double();
  this->get_parameter("gridmap.origin_x", gridmap_origin_x_);
  this->get_parameter("gridmap.origin_y", gridmap_origin_y_);
  this->get_parameter("maze.density", maze_density_);
  this->get_parameter("max_trial_count", max_trial_count_);
  this->get_parameter("simulation_timeout", timeout_);
  this->get_parameter("robot_pose_topic", robot_pose_topic_);
  this->get_parameter("robot_velocity_topic", robot_velocity_topic_);
  this->get_parameter("robot_count", robot_count_);
  this->get_parameter("start_position.x", start_position_.x());
  this->get_parameter("start_position.y", start_position_.y());
  this->get_parameter("start_position.z", start_position_.z());
  this->get_parameter("start_position.yaw", start_yaw_);

  // 得たParmsを表示
  RCLCPP_INFO(
    this->get_logger(),
    "Obstacle Mode: %s, Gridmap Resolution: %f, Gridmap Size: %f x %f, "
    "Gridmap Origin: %f, %f, "
    "Maze Density: %f, Max Trial Count: %d, Simulation Timeout: %f",
    obstacle_mode_.c_str(), resolution_, gridmap_x, gridmap_y, gridmap_origin_x_, gridmap_origin_y_,
    maze_density_, max_trial_count_, timeout_);

  width_ = static_cast<int>(gridmap_x / resolution_);
  height_ = static_cast<int>(gridmap_y / resolution_);

  default_color_.r = 1.0;
  default_color_.g = 1.0;
  default_color_.b = 1.0;
  default_color_.a = 1.0;

  rclcpp::QoS qos_settings(10);
  qos_settings.durability(rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  occupancy_grid_publisher_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", qos_settings);

  slam_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("slam_gridmap", 10);
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(robot_pose_topic_, 10);
  text_marker_publisher_ =
    this->create_publisher<visualization_msgs::msg::Marker>("text_marker", 10);

  twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    robot_velocity_topic_, 10,
    std::bind(&OccupancyMazeSimulator::twist_callback, this, std::placeholders::_1));

  reset_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
    "reset_from_rviz", 10,
    std::bind(&OccupancyMazeSimulator::reset_callback, this, std::placeholders::_1));

  reset_publisher_ = this->create_publisher<std_msgs::msg::Empty>("reset_from_error_manager", 10);

  failed_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "failed", 10, std::bind(&OccupancyMazeSimulator::failed_callback, this, std::placeholders::_1));

  emergency_stop_publisher_ = this->create_publisher<std_msgs::msg::Empty>("emergency_stop", 10);

  pointcloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("glim_ros/points", 10);

  load_map_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

  selected_gridmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "selected_gridmap", 10,
    std::bind(&OccupancyMazeSimulator::selected_gridmap_callback, this, std::placeholders::_1));

  start_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "start_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      start_pose_ = *msg;
      start_pose_received_ = true;
    });

  target_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "target_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      target_pose_ = *msg;
      target_pose_received_ = true;
    });

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  start_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  wait_for_messages();

  if (obstacle_mode_ == "select") {
    RCLCPP_INFO(this->get_logger(), "Select mode, loading selected gridmap.");
    // nav2_map_serverのマップを読み込み
    auto load_map_request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    const char * home_dir = std::getenv("HOME");
    if (home_dir == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't get the home directory.");
    }
    std::string home_directory(home_dir);
    std::string relative_path = "/.ros/save/selected_gridmap";
    full_path_selected_gridmap_filename_ = home_directory + relative_path;
    load_map_request->map_url = full_path_selected_gridmap_filename_ + ".yaml";
    while (!load_map_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          this->get_logger(), "Interrupted while waiting for the load_map service. Exiting.");
        break;
      }
      RCLCPP_INFO(this->get_logger(), "load_map service not available, waiting again...");
    }

    auto load_map_result = load_map_client_->async_send_request(load_map_request);
    if (
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), load_map_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
      grid_map_ = load_map_result.get()->map;

      RCLCPP_INFO(this->get_logger(), "Received gridmap data from map_server.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service load_map");
    }
  }

  reset_callback(std_msgs::msg::Empty::SharedPtr());

  // 起動時にタイムスタンプを用いてCSVファイルを作成。
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  std::string timestamp = ss.str();

  csv_stat_file_name_ = "data_statistics_" + timestamp + ".csv";

  const std::string headers[] = {
    "Trial Count", "Is Reached To Goal",    "Is Failed",     "Failed Msg",
    "Travel Time", "Average Travel Time",   "Average Speed", "Max Speed",
    "Min Speed",   "Min Distance to Object"};

  std::ofstream csv_file(csv_stat_file_name_);
  if (!csv_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create CSV file: %s", csv_stat_file_name_.c_str());
    return;
  }
  csv_file << headers[0];
  for (size_t i = 1; i < std::size(headers); ++i) {
    csv_file << "," << headers[i];
  }
  csv_file << "\n";
  csv_file.close();
}

void OccupancyMazeSimulator::reset_callback(std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_INFO(this->get_logger(), "Resetting the simulation environment.");
  last_update_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();

  std::vector<Obstacle> obstacles;
  int count = 0;
  bool path_exists = false;
  do {
    if (obstacle_mode_ == "maze") {
      obstacles = generate_maze_obstacles();
    } else if (obstacle_mode_ == "random") {
      obstacles = generate_random_obstacles(10);
    } else if (obstacle_mode_ == "select") {
      break;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid obstacle mode: %s", obstacle_mode_.c_str());
      return;
    }
    grid_map_ = create_grid_map(obstacles);
    path_exists = is_path_to_target(grid_map_, start_pose_, target_pose_);
    if (!path_exists) {
      RCLCPP_WARN(this->get_logger(), "No path to the goal exists. Regenerating obstacles.");
      ++count;
    }
    if (count >= 100) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't create valid obstacles. Check your parameters.");
      return;
    }
  } while (!path_exists);

  RCLCPP_INFO(this->get_logger(), "A path to the goal exists.");
  // publish_gridmap_timer_ = this->create_wall_timer(
  //   std::chrono::seconds(1), std::bind(&OccupancyMazeSimulator::publish_gridmap, this));
  publish_gridmap();

  slam_grid_map_ = create_empty_grid_map();

  publish_slam_gridmap_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&OccupancyMazeSimulator::publish_slam_gridmap, this));

  publish_pose_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&OccupancyMazeSimulator::publish_pose, this));

  yaw_ = start_yaw_;
  robot_x_ = start_position_.x();
  robot_y_ = start_position_.y();

  current_linear_velocity_ = 0.0;
  current_angular_velocity_ = 0.0;

  travel_speeds_.clear();
  max_speed_ = 0.0;
  min_speed_ = std::numeric_limits<double>::max();
  min_distance_to_object_ = std::numeric_limits<double>::max();
  start_time_ = rclcpp::Clock(RCL_STEADY_TIME).now();
  is_reached_to_target_ = false;

  reset_publisher_->publish(std_msgs::msg::Empty());
}

void OccupancyMazeSimulator::failed_callback(std_msgs::msg::String::SharedPtr msg)
{
  std::string failed_msg = msg->data;
  RCLCPP_ERROR(this->get_logger(), "Failed: %s", failed_msg.c_str());
  geometry_msgs::msg::Pose text_display_pose;
  text_display_pose.position.x = robot_x_;
  text_display_pose.position.y = robot_y_;

  std_msgs::msg::ColorRGBA text_color;
  text_color = default_color_;
  text_color.g = 0.0;
  text_color.b = 0.0;
  publish_text_marker("Failed:" + failed_msg, text_display_pose, text_color);
  record_statistics(failed_msg);
  reset_callback(std_msgs::msg::Empty::SharedPtr());
}

nav_msgs::msg::OccupancyGrid OccupancyMazeSimulator::create_grid_map(
  const std::vector<Obstacle> & obstacles)
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;
  grid_msg.info.origin.position.x = gridmap_origin_x_;
  grid_msg.info.origin.position.y = gridmap_origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.header.frame_id = "odom";
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.data.resize(width_ * height_, 0);

  RCLCPP_INFO(this->get_logger(), "origin: (%f, %f)", gridmap_origin_x_, gridmap_origin_y_);

  // 障害物の配置
  for (const auto & obstacle : obstacles) {
    // obstacle x,yはgridmapの座標系であって、0~width_, 0~height_の範囲に収まっている。
    int cell_x = static_cast<int>(std::round(obstacle.x / resolution_));
    int cell_y = static_cast<int>(std::round(obstacle.y / resolution_));
    if (cell_x >= 0 && cell_x < width_ && cell_y >= 0 && cell_y < height_) {
      int index = cell_y * width_ + cell_x;
      grid_msg.data[index] = 100;  // 障害物セルを設定
    } else {
      RCLCPP_WARN(this->get_logger(), "Obstacle out of grid bounds at (%d, %d)", cell_x, cell_y);
    }
  }
  return grid_msg;
}

nav_msgs::msg::OccupancyGrid OccupancyMazeSimulator::create_empty_grid_map()
{
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;
  grid_msg.info.origin.position.x = gridmap_origin_x_;
  grid_msg.info.origin.position.y = gridmap_origin_y_;
  grid_msg.info.origin.position.z = 0.0;
  grid_msg.info.origin.orientation.w = 1.0;
  grid_msg.header.frame_id = "odom";
  grid_msg.header.stamp = this->get_clock()->now();
  grid_msg.data.resize(width_ * height_, 0);

  return grid_msg;
}

void OccupancyMazeSimulator::selected_gridmap_callback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received selected grid map, saving it to file...");

  nav2_map_server::SaveParameters save_parameters;
  save_parameters.map_file_name = full_path_selected_gridmap_filename_;
  save_parameters.image_format = "pgm";                  // "pgm", "png"
  save_parameters.mode = nav2_map_server::MapMode::Raw;  // Trinary, Scale, Raw

  try {
    nav2_map_server::saveMapToFile(*msg, save_parameters);
    RCLCPP_INFO(this->get_logger(), "Map saved successfully.");
    std::ifstream f(full_path_selected_gridmap_filename_ + ".yaml");
    if (!f.good()) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to save map, file does not exist: %s",
        (full_path_selected_gridmap_filename_ + ".yaml").c_str());
      return;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
    return;
  }
  grid_map_ = *msg;
  publish_gridmap();
  RCLCPP_INFO(this->get_logger(), "Received selected gridmap data.");
}

bool OccupancyMazeSimulator::is_path_to_target(
  const nav_msgs::msg::OccupancyGrid & grid_map, geometry_msgs::msg::PoseStamped & start,
  geometry_msgs::msg::PoseStamped & target) const
{
  const int width = grid_map.info.width;
  const int height = grid_map.info.height;
  const auto & data = grid_map.data;

  std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
  std::queue<std::pair<int, int>> queue;

  int start_x =
    static_cast<int>((start.pose.position.x - gridmap_origin_x_) / grid_map.info.resolution);
  int start_y =
    static_cast<int>((start.pose.position.y - gridmap_origin_y_) / grid_map.info.resolution);
  int target_x =
    static_cast<int>((target.pose.position.x - gridmap_origin_x_) / grid_map.info.resolution);
  int target_y =
    static_cast<int>((target.pose.position.y - gridmap_origin_y_) / grid_map.info.resolution);

  queue.emplace(start_x, start_y);
  visited[start_y][start_x] = true;

  const int dx[] = {1, -1, 0, 0};
  const int dy[] = {0, 0, 1, -1};

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    if (x == target_x && y == target_y) {
      return true;
    }

    for (int i = 0; i < 4; ++i) {
      int nx = x + dx[i];
      int ny = y + dy[i];

      if (
        nx >= 0 && nx < width && ny >= 0 && ny < height && data[ny * width + nx] == 0 &&
        !visited[ny][nx]) {
        queue.emplace(nx, ny);
        visited[ny][nx] = true;
      }
    }
  }
  return false;
}

Obstacle OccupancyMazeSimulator::create_obstacle(
  double x, double y, double width, double height, double angle)
{
  return Obstacle{x, y, width, height, angle};
}

std::vector<Obstacle> OccupancyMazeSimulator::generate_random_obstacles(int num_obstacles) const
{
  std::vector<Obstacle> obstacles;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist_x(6, width_ - 6);
  std::uniform_real_distribution<> dist_y(6, height_ - 6);
  std::uniform_real_distribution<> dist_width(5, 10);
  std::uniform_real_distribution<> dist_height(5, 10);
  std::uniform_real_distribution<> dist_angle(0, 180);

  obstacles.reserve(num_obstacles);
  for (int i = 0; i < num_obstacles; ++i) {
    obstacles.push_back(create_obstacle(
      dist_x(gen), dist_y(gen), dist_width(gen), dist_height(gen), dist_angle(gen)));
  }
  return obstacles;
}

std::vector<Obstacle> OccupancyMazeSimulator::generate_maze_obstacles() const
{
  std::vector<Obstacle> obstacles;
  int num_cells_x = width_;
  int num_cells_y = height_;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist(0.0, 1.0);  // 0.0～1.0の乱数生成

  for (int i = 0; i < num_cells_x; ++i) {
    for (int j = 0; j < num_cells_y; ++j) {
      if (
        (i * resolution_ <= 10 && j * resolution_ <= 10) ||
        (width_ - 10 <= i * resolution_ && height_ - 10 <= j * resolution_)) {
        continue;
      }

      if (dist(gen) < maze_density_) {  // `maze_density`の確率で障害物を配置
        double x = i * resolution_ + static_cast<double>(resolution_) / 2.0;
        double y = j * resolution_ + static_cast<double>(resolution_) / 2.0;
        double width = (dist(gen) > 0.5) ? resolution_ : resolution_ + 2;
        double height = (width == resolution_) ? resolution_ + 2 : resolution_;
        obstacles.push_back(create_obstacle(x, y, width, height, 0));
      }
    }
  }
  return obstacles;
}

void OccupancyMazeSimulator::publish_pose()
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->get_clock()->now();
  pose_msg.header.frame_id = "odom";
  pose_msg.pose.position.x = robot_x_;
  pose_msg.pose.position.y = robot_y_;
  pose_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";

  transform.transform.translation.x = robot_x_;
  transform.transform.translation.y = robot_y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform);

  auto current_time = rclcpp::Clock(RCL_STEADY_TIME).now();
  double elapsed_time = (current_time - start_time_).seconds();
  if (elapsed_time > timeout_) {
    RCLCPP_INFO(
      this->get_logger(), "Time exceeded %f seconds. Resetting the simulation.", timeout_);
    publish_text_marker(
      "Time exceeded " + std::to_string(timeout_) + " seconds. Resetting the simulation.",
      target_pose_.pose, default_color_);
    std::string timeout_msg = "Simulation timeout";
    record_statistics(timeout_msg);
    reset_callback(std_msgs::msg::Empty::SharedPtr());
  }

  pose_publisher_->publish(pose_msg);
}

void OccupancyMazeSimulator::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Received Twist message: linear=%f, angular=%f", msg->linear.x,
    msg->angular.z);
  simulate_robot_position(msg);
  // simulate_drone_movement(msg);

  // Evaluation Robot位置がゴールに十分近いかどうか
  // 近ければReset
  if (
    robot_x_ > target_pose_.pose.position.x - 0.1 &&
    robot_x_ < target_pose_.pose.position.x + 0.1 &&
    robot_y_ > target_pose_.pose.position.y - 0.1 &&
    robot_y_ < target_pose_.pose.position.y + 0.1) {
    is_reached_to_target_ = true;
    // Targetの位置にText表示
    publish_text_marker(
      "Robot reached the goal. Resetting the simulation.", target_pose_.pose, default_color_);
    record_statistics();
    reset_callback(std_msgs::msg::Empty::SharedPtr());
  }
}

void OccupancyMazeSimulator::publish_gridmap()
{
  occupancy_grid_publisher_->publish(grid_map_);
  RCLCPP_INFO(this->get_logger(), "Trial Count: %d", trial_count_);
}

// Default option for robot position calculation
void OccupancyMazeSimulator::simulate_robot_position(geometry_msgs::msg::Twist::SharedPtr msg)
{
  auto current_time = rclcpp::Clock(RCL_STEADY_TIME).now();

  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  // Update position based on both x and y velocities and orientation
  // double delta_x =
  //   msg->linear.x * std::cos(yaw_ - M_PI / 4) * dt - msg->linear.y * std::sin(yaw_ - M_PI / 4) *
  //   dt;
  // double delta_y =
  //   msg->linear.x * std::sin(yaw_ - M_PI / 4) * dt + msg->linear.y * std::cos(yaw_ - M_PI / 4) *
  //   dt;
  // double delta_yaw = msg->angular.z * dt;
  double delta_x = msg->linear.x * dt;
  double delta_y = msg->linear.y * dt;
  double delta_yaw = msg->angular.z * dt;

  robot_x_ += delta_x;
  robot_y_ += delta_y;
  yaw_ += delta_yaw;

  if (yaw_ > M_PI) yaw_ -= 2 * M_PI;
  if (yaw_ < -M_PI) yaw_ += 2 * M_PI;

  double speed = std::sqrt(std::pow(msg->linear.x, 2) + std::pow(msg->linear.y, 2));
  travel_speeds_.push_back(speed);
  max_speed_ = std::max(max_speed_, speed);
  min_speed_ = std::min(min_speed_, speed);

  // TODO(Izumita): #15 Implement 最小距離の計算
  // // 物体との距離を計算し、最小距離を更新
  // double distance_to_object = calculate_distance_to_object();
  // min_distance_to_object_ = std::min(min_distance_to_object_, distance_to_object);

  // // 物体とのHit回数を更新
  // if (distance_to_object < hit_threshold_) {
  //   hit_count_++;
  // }

  RCLCPP_DEBUG(
    this->get_logger(), "Updated Robot Position: [x: %f, y: %f, yaw: %f]", robot_x_, robot_y_,
    yaw_);
}

void OccupancyMazeSimulator::simulate_lidar_scan()
{
  const int num_lidar_rays = 360;       // 360本のレーザー
  const double max_lidar_range = 10.0;  // LiDARの最大範囲 (メートル)
  const double angle_increment = 2 * M_PI / num_lidar_rays;

  for (int i = 0; i < num_lidar_rays; ++i) {
    double angle = i * angle_increment + yaw_;
    double end_x = robot_x_ + max_lidar_range * std::cos(angle);
    double end_y = robot_y_ + max_lidar_range * std::sin(angle);

    // グリッド座標に変換
    int start_cell_x = static_cast<int>(std::round((robot_x_ - gridmap_origin_x_) / resolution_));
    int start_cell_y = static_cast<int>(std::round((robot_y_ - gridmap_origin_y_) / resolution_));
    int end_cell_x = static_cast<int>(std::round((end_x - gridmap_origin_x_) / resolution_));
    int end_cell_y = static_cast<int>(std::round((end_y - gridmap_origin_y_) / resolution_));

    // Bresenhamのアルゴリズムを適用
    int dx = std::abs(end_cell_x - start_cell_x);
    int dy = std::abs(end_cell_y - start_cell_y);
    int sx = (start_cell_x < end_cell_x) ? 1 : -1;
    int sy = (start_cell_y < end_cell_y) ? 1 : -1;
    int err = dx - dy;

    int current_x = start_cell_x;
    int current_y = start_cell_y;

    while (true) {
      // 範囲外チェック
      if (current_x < 0 || current_x >= width_ || current_y < 0 || current_y >= height_) {
        break;
      }

      int index = current_y * width_ + current_x;

      // 障害物セルに到達した場合、SLAMマップを更新して終了
      if (grid_map_.data[index] == 100) {
        slam_grid_map_.data[index] = 100;
        break;
      }

      // 空きセルの場合、SLAMマップを非占有のまま
      slam_grid_map_.data[index] = 0;

      // ゴールに到達した場合
      if (current_x == end_cell_x && current_y == end_cell_y) {
        break;
      }

      // 次のセルへ進む
      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        current_x += sx;
      }
      if (e2 < dx) {
        err += dx;
        current_y += sy;
      }
    }
  }
}

void OccupancyMazeSimulator::publish_slam_gridmap()
{
  simulate_lidar_scan();
  generate_and_publish_pointcloud();
  slam_grid_map_.header.stamp = this->get_clock()->now();
  slam_grid_publisher_->publish(slam_grid_map_);
}

void OccupancyMazeSimulator::record_statistics(std::string failed_msg)
{
  record_count_++;
  if (record_count_ >= max_trial_count_) {
    RCLCPP_INFO(this->get_logger(), "Recorded %d trials. Exiting the simulation.", record_count_);
    emergency_stop_publisher_->publish(std_msgs::msg::Empty());
    rclcpp::shutdown();
  }
  bool is_failed = false;
  if (failed_msg != "") {
    is_failed = true;
  }
  trial_count_++;
  double travel_time = (rclcpp::Clock(RCL_STEADY_TIME).now() - start_time_).seconds();

  if (is_reached_to_target_) {
    RCLCPP_INFO(this->get_logger(), "Reached to the target in %f seconds.", travel_time);
    travel_times_.push_back(travel_time);
  }

  double average_speed = 0.0;
  if (!travel_speeds_.empty()) {
    average_speed =
      std::accumulate(travel_speeds_.begin(), travel_speeds_.end(), 0.0) / travel_speeds_.size();
  }
  double average_travel_time = NAN;
  if (!travel_times_.empty()) {
    average_travel_time =
      std::accumulate(travel_times_.begin(), travel_times_.end(), 0.0) / travel_times_.size();
  }
  std::ofstream csv_file(csv_stat_file_name_, std::ios::app);
  csv_file << trial_count_ << "," << is_reached_to_target_ << "," << is_failed << "," << failed_msg
           << "," << travel_time << "," << average_travel_time << "," << average_speed << ","
           << max_speed_ << "," << min_speed_ << "," << min_distance_to_object_ << "\n";
  csv_file.close();
}

void OccupancyMazeSimulator::publish_text_marker(
  std::string visualize_text, geometry_msgs::msg::Pose marker_pose,
  std_msgs::msg::ColorRGBA text_color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = this->now();
  marker.ns = "text_marker";
  marker.id = 999;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.z = 2.0;
  marker.pose = marker_pose;

  marker.color = text_color;

  marker.lifetime = rclcpp::Duration::from_seconds(5);

  marker.text = visualize_text;

  text_marker_publisher_->publish(marker);
}

void OccupancyMazeSimulator::wait_for_messages()
{
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && (!start_pose_received_ || !target_pose_received_)) {
    RCLCPP_INFO(this->get_logger(), "Waiting for messages...");
    rclcpp::spin_some(this->get_node_base_interface());
    publish_pose();
    rate.sleep();
  }
}

void OccupancyMazeSimulator::generate_and_publish_pointcloud()
{
  // PointCloud2メッセージを準備
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header.stamp = this->get_clock()->now();
  pointcloud_msg.header.frame_id = "odom";

  // PointCloud2メッセージのフィールドを設定
  pointcloud_msg.height = 1;  // PointCloud2は1Dデータとして送信
  pointcloud_msg.is_bigendian = false;
  pointcloud_msg.is_dense = true;

  // 点群データキャッシュ用
  std::vector<float> points_x;
  std::vector<float> points_y;
  std::vector<float> points_z;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dist_pos(0, slam_grid_map_.info.resolution);
  std::uniform_int_distribution<> dist_points(5, 10);

  for (unsigned int y = 0; y < slam_grid_map_.info.height; ++y) {
    for (unsigned int x = 0; x < slam_grid_map_.info.width; ++x) {
      int index = y * slam_grid_map_.info.width + x;

      if (slam_grid_map_.data[index] == 100) {  // 占有セル
        // 既にキャッシュされている場合はスキップ
        if (cell_pointcloud_cache_.find(index) != cell_pointcloud_cache_.end()) {
          const auto &cell_points = cell_pointcloud_cache_[index];
          for (const auto &p : cell_points) {
            points_x.push_back(p.x);
            points_y.push_back(p.y);
            points_z.push_back(p.z);
          }
          continue;
        }

        // 新規点群を生成
        double cell_origin_x =
          slam_grid_map_.info.origin.position.x + x * slam_grid_map_.info.resolution;
        double cell_origin_y =
          slam_grid_map_.info.origin.position.y + y * slam_grid_map_.info.resolution;

        int num_points = dist_points(gen);

        std::vector<geometry_msgs::msg::Point> cell_points;
        for (int i = 0; i < num_points; ++i) {
          geometry_msgs::msg::Point point;
          point.x = cell_origin_x + dist_pos(gen);
          point.y = cell_origin_y + dist_pos(gen);
          point.z = 0.0;  // Z軸は0に固定

          points_x.push_back(point.x);
          points_y.push_back(point.y);
          points_z.push_back(point.z);

          cell_points.push_back(point);
        }

        // キャッシュに保存
        cell_pointcloud_cache_[index] = cell_points;
      }
    }
  }

  // PointCloud2のデータ構造に変換
  size_t num_points = points_x.size();
  pointcloud_msg.width = static_cast<uint32_t>(num_points);
  pointcloud_msg.row_step = pointcloud_msg.width * sizeof(float) * 3;  // x, y, zの3要素
  pointcloud_msg.point_step = sizeof(float) * 3;

  // フィールド定義
  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  // データを埋める
  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
  for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    *iter_x = points_x[i];
    *iter_y = points_y[i];
    *iter_z = points_z[i];
  }

  pointcloud_publisher_->publish(pointcloud_msg);
}

// Alt option for robot position simulation Not TESTED, so comment out for now
// void OccupancyMazeSimulator::simulate_drone_movement(
//   geometry_msgs::msg::Twist::SharedPtr target_twist)
// {
//   // シミュレーションのパラメータ
//   const double max_linear_acceleration = 0.5;   // 最大線形加速度 (m/s^2)
//   const double max_angular_acceleration = 0.5;  // 最大角加速度 (rad/s^2)
//   const double drift_noise_std_dev = 0.05;      // ドリフトノイズの標準偏差

//   auto wall_clock = rclcpp::Clock(RCL_STEADY_TIME);
//   auto current_time = wall_clock.now();

//   double dt = (current_time - last_update_time_).seconds();
//   last_update_time_ = current_time;

//   // 目標の線形速度と角速度
//   double target_linear_x = target_twist->linear.x;
//   double target_angular_z = target_twist->angular.z;

//   // 慣性を考慮して、現在の速度を目標速度に段階的に近づける
//   double linear_acceleration = std::clamp(
//     (target_linear_x - current_linear_velocity_) / dt, -max_linear_acceleration,
//     max_linear_acceleration);
//   double angular_acceleration = std::clamp(
//     (target_angular_z - current_angular_velocity_) / dt, -max_angular_acceleration,
//     max_angular_acceleration);

//   current_linear_velocity_ += linear_acceleration * dt;
//   current_angular_velocity_ += angular_acceleration * dt;

//   // 滑り（ドリフト）のシミュレーション - 正規分布ノイズを追加
//   std::random_device rd;
//   std::mt19937 gen(rd());
//   std::normal_distribution<> drift_noise(0.0, drift_noise_std_dev);
//   double drift_x = drift_noise(gen);
//   double drift_y = drift_noise(gen);

//   // 新しい位置を計算
//   double delta_x = (current_linear_velocity_ * std::cos(yaw_) + drift_x) * dt;
//   double delta_y = (current_linear_velocity_ * std::sin(yaw_) + drift_y) * dt;
//   double delta_yaw = current_angular_velocity_ * dt;

//   // 位置と姿勢の更新
//   robot_x_ += delta_x;
//   robot_y_ += delta_y;
//   yaw_ += delta_yaw;

//   // PoseStampedメッセージの更新
//   geometry_msgs::msg::PoseStamped pose_msg;
//   pose_msg.header.stamp = current_time;
//   pose_msg.header.frame_id = "odom";
//   pose_msg.pose.position.x = robot_x_;
//   pose_msg.pose.position.y = robot_y_;
//   pose_msg.pose.position.z = 0.0;

//   // Quaternionを用いてYawからOrientationを設定
//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw_);
//   pose_msg.pose.orientation.x = q.x();
//   pose_msg.pose.orientation.y = q.y();
//   pose_msg.pose.orientation.z = q.z();
//   pose_msg.pose.orientation.w = q.w();

//   // 更新された位置をパブリッシュ
//   pose_publisher_->publish(pose_msg);
// }

}  // namespace occupancy_maze_simulator

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(occupancy_maze_simulator::OccupancyMazeSimulator)
