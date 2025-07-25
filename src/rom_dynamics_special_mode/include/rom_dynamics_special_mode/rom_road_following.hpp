#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <algorithm>

static const std::string kName = "ROM RoadFollow";

using namespace px4_ros2::literals; // NOLINT

class RoadFollowing : public px4_ros2::ModeBase
{
public:
  explicit RoadFollowing(rclcpp::Node & node) : ModeBase(node, kName)
  {
    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);

    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
  }

  void onActivate() override
  {
    _state = State::SettlingAtStart;
    _start_position_set = false;
    _start_heading_set = false;
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    if (!_start_position_set) {
      _start_position_m = _vehicle_local_position->positionNed();
      _start_position_set = true;
    }

    switch (_state) {
      case State::SettlingAtStart: {
          // just settling at the starting vehicle position
          _goto_setpoint->update(_start_position_m);
          if (positionReached(_start_position_m)) {
            _state = State::GoingNorth;
          }
        }
        break;

      case State::GoingNorth: {
        // Define the target position in NED (North-East-Down) coordinates.
        // We're moving North, so the X component (North) increases by kTriangleHeight.
        const Eigen::Vector3f target_position_m = _start_position_m + Eigen::Vector3f{kTriangleHeight, 0.f, 0.f}; // NED, XYZ

        // Calculate the vector from the vehicle's current position to the target in the horizontal plane (XY).
        const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) - _vehicle_local_position->positionNed().head(2);

        // Determine the desired heading to face the target.
        // atan2f gives the angle in radians between the positive x-axis and the point (x, y).
        // Here, x is the North difference, and y is the East difference.
        const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

        // Define a proportional gain for velocity control.
        // This value determines how aggressively the drone will move towards the target.
        // You might need to tune this (e.g., to 0.5f, 1.0f, etc.) based on your drone's dynamics.
        const float kp_vel = 0.7f; // Proportional gain for velocity
        // Define a maximum linear speed to cap the commanded velocity.
        const float max_linear_speed = 2.0f; // m/s

        // Calculate the desired linear velocity towards the target.
        // We normalize the vehicle_to_target_xy vector to get a direction,
        // then multiply by kp_vel and min(distance_to_target, max_linear_speed)
        // to ensure we slow down as we approach the target and don't exceed max speed.
        Eigen::Vector3f desired_velocity_ned = Eigen::Vector3f::Zero();
        float distance_to_target_xy = vehicle_to_target_xy.norm();

        if (distance_to_target_xy > 0.1f) { // If far enough from target, calculate velocity
            desired_velocity_ned.head(2) = kp_vel * vehicle_to_target_xy.normalized() * std::min(distance_to_target_xy, max_linear_speed);
        } else {
            // If very close to the target, set velocity to zero to stop.
            desired_velocity_ned.head(2).setZero();
        }

        // Update the setpoint with the desired velocity and heading.
        // The 'z' component of velocity (downwards) can be controlled separately for altitude.
        // For this example, we keep vertical velocity at 0, assuming altitude is managed by another controller or is constant.
        _goto_setpoint->update(desired_velocity_ned, heading_target_rad);


        // Check if the position has been reached.
        // This condition remains crucial for state transitions.
        if (positionReached(target_position_m)) {
            // Optionally, send a zero velocity command one last time to ensure a complete stop
            // before transitioning, or let the next state's logic handle it.
            _goto_setpoint->update(Eigen::Vector3f::Zero(), heading_target_rad);
            _state = State::GoingEast;
        }
        }
        break;
    }
  }

private:
  static constexpr float kTriangleHeight = 20.f; // [m]
  static constexpr float kTriangleWidth = 30.f; // [m]

  enum class State
  {
    SettlingAtStart = 0,
    FollowingRoad
  } _state;

  // NED earth-fixed frame. box pattern starting corner (first position the mode sees on activation)
  Eigen::Vector3f _start_position_m;
  bool _start_position_set{false};

  // [-pi, pi] current heading setpoint during spinning phase
  float _spinning_heading_rad{0.f};

  // used for heading initialization when dynamically updating heading setpoints
  bool _start_heading_set{false};

  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  bool positionReached(const Eigen::Vector3f & target_position_m) const
  {
    static constexpr float kPositionErrorThreshold = 0.5f; // [m]
    static constexpr float kVelocityErrorThreshold = 0.3f; // [m/s]
    const Eigen::Vector3f position_error_m = target_position_m -
      _vehicle_local_position->positionNed();
    return (position_error_m.norm() < kPositionErrorThreshold) &&
           (_vehicle_local_position->velocityNed().norm() < kVelocityErrorThreshold);
  }

  bool headingReached(float target_heading_rad) const
  {
    static constexpr float kHeadingErrorThreshold = 7.0_deg;
    const float heading_error_wrapped = px4_ros2::wrapPi(
      target_heading_rad - _vehicle_local_position->heading());
    return fabsf(heading_error_wrapped) < kHeadingErrorThreshold;
  }
};
