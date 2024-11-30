/*
  Copyright (c) 2024 Masashi Izumita

  All rights reserved.
 */
#ifndef BUTTON_PANEL_HPP_
#define BUTTON_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>

#include <qspinbox.h>

#ifndef Q_MOC_RUN
#include <QtWidgets>
#include <rviz_common/panel.hpp>
#endif

namespace occupancy_maze_simulator_rviz_plugins
{
class ButtonPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ButtonPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void on_reset_button_clicked();

  QPushButton * reset_button_ = nullptr;

  QVBoxLayout * panel_vertical_layout_ = nullptr;
  QHBoxLayout * panel_horizontal_layout_ = nullptr;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;
};
}  // namespace occupancy_maze_simulator_rviz_plugins

#endif  // BUTTON_PANEL_HPP_
