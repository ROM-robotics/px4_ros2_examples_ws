#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <algorithm>

static const std::string kName = "ROM TriangleLand";

using namespace px4_ros2::literals; // NOLINT

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node) : ModeBase(node, kName)
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
      // case State::Reset:
      //   break;

      // case State::TakingOff: {
      //     // Define a target altitude for takeoff. This will be relative to the initial ground Z position.
      //     static constexpr float kTakeoffAltitude = 5.0f; // meters above the start Z position (NED: negative Z is up)
          
      //     // Calculate the target takeoff position. We only change the Z component.
      //     // In NED, a higher altitude means a more negative Z value.
      //     const Eigen::Vector3f takeoff_target_position_m = 
      //         Eigen::Vector3f(_start_position_m.x(), _start_position_m.y(), _start_position_m.z() - kTakeoffAltitude);

      //     _goto_setpoint->update(takeoff_target_position_m);

      //     // Check if the takeoff altitude is reached and the vehicle is relatively stable
      //     if (altitudeReached(takeoff_target_position_m.z()) && _vehicle_local_position->velocityNed().norm() < 0.5f) {
      //       //RCLCPP_INFO(this->get_logger(), "Takeoff complete. Reached altitude: %f", kTakeoffAltitude);
      //       _state = State::SettlingAtStart;
      //     }
      //   }
      //   break;

      case State::SettlingAtStart: {
          // just settling at the starting vehicle position
          _goto_setpoint->update(_start_position_m);
          if (positionReached(_start_position_m)) {
            _state = State::GoingNorth;
          }
        }
        break;

      case State::GoingNorth: {
          // go north to the northwest corner facing in direction of travel
          const Eigen::Vector3f target_position_m = _start_position_m + Eigen::Vector3f{kTriangleHeight, 0.f, 0.f}; // NED, XYZ

          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) - _vehicle_local_position->positionNed().head(2);
          const float heading_target_rad = atan2f(vehicle_to_target_xy(1), vehicle_to_target_xy(0));

          if (vehicle_to_target_xy.norm() < 0.1f) {
            // stop caring about heading (the arctangent becomes undefined)
            _goto_setpoint->update(target_position_m);
          } else {
            _goto_setpoint->update(target_position_m, heading_target_rad);
          }

          if (positionReached(target_position_m)) {
            _state = State::GoingEast;
          }
        }
        break;

      case State::GoingEast: {
          // go to the northeast corner while spinning
          const Eigen::Vector3f target_position_m = _start_position_m + Eigen::Vector3f{kTriangleHeight, kTriangleWidth, 0.f};

          // scale the speed limits by distance to the target
          const Eigen::Vector2f vehicle_to_target_xy = target_position_m.head(2) - _vehicle_local_position->positionNed().head(2);
          const float speed_scale = std::min(vehicle_to_target_xy.norm() / kTriangleWidth, 1.f);

          const float max_horizontal_velocity_m_s = 5.f * speed_scale + (1.f - speed_scale) * 1.f;
          const float max_vertical_velocity_m_s = 3.f * speed_scale + (1.f - speed_scale) * 0.5f;

          const float max_heading_rate_rad_s = px4_ros2::degToRad(45.f * speed_scale + (1.f - speed_scale) * 25.f);
          const float heading_setpoint_rate_of_change = px4_ros2::degToRad(40.f * speed_scale + (1.f - speed_scale) * 20.f);

          if (!_start_heading_set) {
            _spinning_heading_rad = _vehicle_local_position->heading();
            _start_heading_set = true;
          }

          if (!positionReached(target_position_m)) {
            _spinning_heading_rad += heading_setpoint_rate_of_change * dt_s;
          }

          _goto_setpoint->update(
            target_position_m,
            _spinning_heading_rad,
            max_horizontal_velocity_m_s,
            max_vertical_velocity_m_s,
            max_heading_rate_rad_s);

          if (positionReached(target_position_m)) {
            _state = State::GoingSouthwest;
          }
        }
        break;

      case State::GoingSouthwest: {
          // go to southwest corner while facing the northwestern corner
          const Eigen::Vector2f position_of_interest_m = _start_position_m.head(2) + Eigen::Vector2f{kTriangleHeight, 0.f};
          const Eigen::Vector2f vehicle_to_poi_xy = position_of_interest_m - _vehicle_local_position->positionNed().head(2);
          
          const float heading_target_rad = atan2f(vehicle_to_poi_xy(1), vehicle_to_poi_xy(0));

          _goto_setpoint->update(_start_position_m, heading_target_rad);
          if (positionReached(_start_position_m)) {
            _state = State::Landing;
          }
        }
        break;

      case State::Landing: {
          const Eigen::Vector3f landing_target_position_m = _start_position_m; 
          
          // Provide explicit values for max horizontal velocity and max heading rate.
          // Adjust these values as needed for a safe and controlled landing.
          // A smaller value for horizontal velocity means it will try to stop horizontal movement more aggressively.
          // A smaller value for heading rate means it will rotate slower.
          static constexpr float kLandingMaxHorizontalVelocity = 1.0f; // meters/second
          static constexpr float kLandingMaxVerticalVelocity = 0.5f;   // meters/second (downward)
          // Calculate 15 degrees to radians directly for constexpr
          static constexpr float kLandingMaxHeadingRate = 15.0f * (M_PI / 180.0f); // radians/second (e.g., 15 degrees/second)

          _goto_setpoint->update(
            landing_target_position_m,
            _vehicle_local_position->heading(), // Maintain current heading during landing
            kLandingMaxHorizontalVelocity, 
            kLandingMaxVerticalVelocity, 
            kLandingMaxHeadingRate);

          if (fabsf(_vehicle_local_position->positionNed().z() - _start_position_m.z()) < kGroundAltitudeThreshold &&
              fabsf(_vehicle_local_position->velocityNed().z()) < kLandedVerticalVelocityThreshold) 
          {
            //RCLCPP_INFO(node_.get_logger(), "Landing complete. Mission success."); // Added log for clarity
            completed(px4_ros2::Result::Success); // Indicate mission completion
            return;
          }
        }
        break;
    }
  }

private:
  static constexpr float kTriangleHeight = 20.f; // [m]
  static constexpr float kTriangleWidth = 30.f; // [m]

  // Thresholds for landing detection
  static constexpr float kGroundAltitudeThreshold = 0.2f; // [m] How close to ground Z to consider landed
  static constexpr float kLandedVerticalVelocityThreshold = 0.1f; // [m/s] How slow vertical velocity to consider landed

  enum class State
  {
    // Reset,
    // TakingOff,
    SettlingAtStart = 0,
    GoingNorth,
    GoingEast,
    GoingSouthwest,
    Landing
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

  bool altitudeReached(float target_altitude_z_ned) const
  {
      static constexpr float kAltitudeErrorThreshold = 0.3f; // [m]
      return fabsf(target_altitude_z_ned - _vehicle_local_position->positionNed().z()) < kAltitudeErrorThreshold;
  }
};
