#include "rclcpp/rclcpp.hpp"

#include <rom_dynamics_special_mode/rom_gotriangle_mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

using MyNodeWithMode = px4_ros2::NodeWithMode<FlightModeTest>;

static const std::string kNodeName = "rom_triangle";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
