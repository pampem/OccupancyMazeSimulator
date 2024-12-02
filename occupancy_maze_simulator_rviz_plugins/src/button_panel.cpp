/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#include "button_panel.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>

namespace occupancy_maze_simulator_rviz_plugins
{
ButtonPanel::ButtonPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  panel_vertical_layout_ = new QVBoxLayout;
  panel_horizontal_layout_ = new QHBoxLayout;

  panel_horizontal_layout_->addLayout(panel_vertical_layout_);

  reset_button_ = new QPushButton("Reset", this);
  panel_horizontal_layout_->addWidget(reset_button_);
  connect(reset_button_, &QPushButton::clicked, this, &ButtonPanel::on_reset_button_clicked);

  setLayout(panel_horizontal_layout_);
}

void ButtonPanel::onInitialize()
{
  try {
    node_ = std::make_shared<rclcpp::Node>("button_panel_node");
    reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("reset_from_rviz", 10);
  } catch (const rclcpp::exceptions::RCLError & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("button_panel"), "Failed to initialize ROS 2 node: %s", e.what());
    throw;
  }
}

void ButtonPanel::on_reset_button_clicked()
{
  auto msg = std_msgs::msg::Empty();
  reset_publisher_->publish(msg);
}

}  // namespace occupancy_maze_simulator_rviz_plugins

PLUGINLIB_EXPORT_CLASS(occupancy_maze_simulator_rviz_plugins::ButtonPanel, rviz_common::Panel)
