#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <Eigen/Core>
// #include <px4_ros2/control/setpoint_types/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

static const std::string kName = "ROM Velocity Example";

class VelocityMode : public px4_ros2::ModeBase
{
public:
  explicit VelocityMode(rclcpp::Node & node) : ModeBase(node, kName)
  {
    //_traj_pub = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    _traj_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override {}

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    Eigen::Vector3f acceleration_ned_m_s2 = Eigen::Vector3f::Zero(); // No acceleration
    float yaw_ned_rad = _vehicle_local_position->heading(); // Keep current heading
    float yaw_rate_ned_rad_s = 0.0f; // No yaw rate change

    // Example: send 1 m/s North, 0.5 m/s East, 0 m/s Down, 0.1 rad/s yaw rate
    //_traj_setpoint->update(Eigen::Vector3f(1.0f, 0.5f, 0.0f), 0.1f);
    _traj_setpoint->update(Eigen::Vector3f(1.0f, 0.5f, 0.0f), acceleration_ned_m_s2, yaw_ned_rad, yaw_rate_ned_rad_s);
     
    // void TrajectorySetpointType::update(
    // const Eigen::Vector3f & velocity_ned_m_s,
    // const std::optional<Eigen::Vector3f> & acceleration_ned_m_s2,
    // std::optional<float> yaw_ned_rad,
    // std::optional<float> yaw_rate_ned_rad_s)

    // px4_msgs::msg::TrajectorySetpoint msg;
    // msg.velocity[0] = 1.0f;   // North
    // msg.velocity[1] = 0.5f;   // East
    // msg.velocity[2] = 0.0f;   // Down
    // msg.yaw = _vehicle_local_position->heading(); // Keep current heading
    // msg.yawspeed = 0.1f;      // Yaw rate (rad/s)
    // _traj_pub->publish(msg);
    //_traj_setpoint->update(Eigen::Vector3f(1.0f, 0.5f, 0.0f), acceleration_ned_m_s2, yaw_ned_rad, yaw_rate_ned_rad_s);
  }

private:
  //rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _traj_pub;
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _traj_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
};
