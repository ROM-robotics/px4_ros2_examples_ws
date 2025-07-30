/****************************************************************************
 * Copyright (c) 2023 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include "rclcpp/rclcpp.hpp"

#include <rom_dynamics_special_mode/autonomous_follow_road_mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithModeExecutor = px4_ros2::NodeWithModeExecutor<ModeExecutorTest, RoadFollowMode>;

static const std::string kNodeName = "ROM_Hacker";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithModeExecutor>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
