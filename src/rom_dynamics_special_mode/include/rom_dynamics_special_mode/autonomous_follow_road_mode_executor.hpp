#include <limits>

// Simple PID controller class
class PID {
public:
  PID(float kp, 
    float ki, 
    float kd, 
    float min_out = -std::numeric_limits<float>::max(), 
    float max_out = std::numeric_limits<float>::max())
    : _kp(kp), _ki(ki), _kd(kd), _min_out(min_out), _max_out(max_out) {}

  void reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
    _first = true;
  }

  float update(float error, float dt) {
    if (_first) {
      _prev_error = error;
      _first = false;
    }
    _integral += error * dt;
    float derivative = (dt > 0.0f) ? (error - _prev_error) / dt : 0.0f;
    float output = _kp * error + _ki * _integral + _kd * derivative;
    output = std::clamp(output, _min_out, _max_out);
    _prev_error = error;
    return output;
  }


  void setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
  }

  void setOutputLimits(float min_out, float max_out) {
    _min_out = min_out;
    _max_out = max_out;
  }

private:
  float _kp, _ki, _kd;
  float _integral = 0.0f;
  float _prev_error = 0.0f;
  float _min_out, _max_out;
  bool _first = true;
};

#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>
#include <algorithm>

#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <optional>


#define ROM_DEBUG 1
#define ROM_UNUSED(x) (void)(x)

static const std::string kName = "ROM Autonomous";

using namespace px4_ros2::literals; // NOLINT
using namespace std::chrono_literals; // NOLINT

class RoadFollowMode : public px4_ros2::ModeBase
{
public:
  explicit RoadFollowMode(rclcpp::Node & node) : ModeBase(node, kName), _node(node)
  {
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _road_line_sub = node.create_subscription<sensor_msgs::msg::Image>(
      "/road_line", 10, std::bind(&RoadFollowMode::roadLineCallback, this, std::placeholders::_1)
    );
    _traj_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
  }

  float getCurrentAltitudeNED() const {
    return _vehicle_local_position ? _vehicle_local_position->positionNed()(2) : 0.0f;
  }

  void onActivate() override {    _state = State::SettlingAtStart;    }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  { 
    switch (_state) {
      case State::SettlingAtStart: {
          if (_latest_road_line_img) {
            _state = State::FollowRoad;
          }
        }
        break;

      case State::FollowRoad: {
        
          if (!_latest_road_line_img) {
            
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            RCLCPP_WARN(_node.get_logger(), "No road line image received, hovering.");
            break;
          }
          
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(_latest_road_line_img, "bgr8");
          } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(_node.get_logger(), "cv_bridge exception: %s", e.what());
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }

          cv::Mat img = cv_ptr->image;
          int H = img.rows, W = img.cols;
          ROM_UNUSED(H); 
          float center_x = W / 2.0f;
          
          #ifdef ROM_DEBUG
            if (!debug_done_step1) 
            {
              debug_done_step1 = true;
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP1 ] Image received: %dx%d", H, W);
            }
          #endif
          
          cv::Mat mask;
          cv::inRange(img, cv::Scalar(0, 0, 151), cv::Scalar(49, 49, 255), mask);
          
          #ifdef ROM_DEBUG
            if (!debug_done_step2) 
            {
              debug_done_step2 = true;
              for (int i = 0; i < std::min(10, mask.rows); ++i) {
                for (int j = 0; j < std::min(10, mask.cols); ++j) {
                    std::cout << (int)mask.at<uchar>(i, j) << " ";
                }
                std::cout << std::endl;
              }
              std::cout << "height: " <<mask.rows << std::endl;
              std::cout << "width: " << mask.cols << std::endl;
            }
          #endif
          
          std::vector<cv::Point> pts;
          cv::findNonZero(mask, pts);
          
          if (pts.empty()) {
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }
          
          cv::Vec4f line_params;
          cv::fitLine(pts, line_params, cv::DIST_L2, 0, 0.01, 0.01);
          float vx_fit = line_params[0], vy_fit = line_params[1];
          
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] Fit line params: vx: %.4f, vy: %.4f", vx_fit, vy_fit);
          #endif
          
          float theta_angle = atan2f(vy_fit, vx_fit);
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] theta_angle_deg %.4f", theta_angle * 57.2958);
          #endif
          
          float desired_angle = M_PI / 2.0f;
          
          float raw_angle_error = fabs(theta_angle) - desired_angle;
          if(theta_angle < 0) {
            
            raw_angle_error *= -1.0f;
          }
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] raw_angle_error %.4f", raw_angle_error * 57.2958);
          #endif

          while (raw_angle_error > M_PI) raw_angle_error -= 2 * M_PI;
          while (raw_angle_error < -M_PI) raw_angle_error += 2 * M_PI;

          _filtered_angle_error = _alpha_angle * raw_angle_error + (1.0f - _alpha_angle) * _filtered_angle_error;
          float angle_error = _filtered_angle_error;
          // ============================================================== angle_error တွက်ပြီးပြီ။
          
          cv::Moments M = cv::moments(mask, true);
          
          float centroid_x = (M.m00 == 0) ? center_x : M.m10 / M.m00;
          
          float error_x = centroid_x - center_x;
          
          float error_x_norm = error_x / (W / 2.0f);
          
          float raw_lateral_vel = _kp_centroid * error_x_norm * _max_lateral_speed;
          
          float lateral_vel = _alpha_vel * raw_lateral_vel + (1.0f - _alpha_vel) * _prev_lateral_vel;
          _prev_lateral_vel = lateral_vel;
          
          float body_forward = 0.0f;
          float body_right = 0.0f;
          float body_yawrate = 0.0f;

          // ================================================================ 
          
          if (fabs(angle_error) > _angle_tolerance) 
          {
            body_forward = 0.0f;
            body_right = 0.0f;
            body_yawrate = _kp_yaw * angle_error; // 0.2 * 0.3491 rad(20 degrees) = 0.06982 rad/s
          } 
          
          else {
            body_forward = _forward_speed;
            body_right = lateral_vel;
            body_yawrate = _kp_yaw * angle_error;
          }
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP3 ] angle_err(deg): %.4f, _kp_yaw: %.4f, yaw_rate(rad/s): %.4f", angle_error * 57.2958, _kp_yaw, body_yawrate);
          #endif

          // Convert body-frame (forward, right) to NED (PX4 expects NED)
          float ψ = _vehicle_local_position->heading(); // ψ က drone ရဲ့ ့ heading, yaw angle
          float u = body_forward;
          float v = body_right;
          float v_north =  u * cosf(ψ) - v * sinf(ψ);
          float v_east  =  u * sinf(ψ) + v * cosf(ψ);

          // ==================== ALTITUDE PID CONTROL ======================= //
          float current_agl_altitude = _vehicle_local_position->distanceGround(); // above ground level always POSITIVE
          float altitude_error = desire_agl_altitude - current_agl_altitude;
          static PID altitude_pid(0.5f, 0.1f, 0.2f, -2.0f, 2.0f); // Example gains, tune as needed
          // Optionally set output limits at runtime:
          altitude_pid.setOutputLimits(-1.5f, 1.5f); // min/max vertical velocity (m/s)
          float vz_cmd = -altitude_pid.update(altitude_error, dt_s); // Positive vz_cmd = up
          #ifdef ROM_DEBUG
            RCLCPP_INFO(_node.get_logger(), "[ ALTIDUDE PID CONTROLLER ] current_altitude: %.2f, error: %.2f, vz_cmd: %.2f", current_agl_altitude, altitude_error, vz_cmd);
          #endif
          // =================== END ALTITUDE CONTROL ====================== //
          
          
          
          setVelocitySetpoint(Eigen::Vector3f{v_north, v_east, vz_cmd}, body_yawrate);
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP3 ] v_north(m/s): %.4f, v_east(m/s): %.4f", v_north, body_right);
          #endif
        }
        break;
    }
  }

  void setVelocitySetpoint(const Eigen::Vector3f & velocity_ned, float yaw_rate)
  {
    #ifdef ROM_DEBUG1
    RCLCPP_INFO(_node.get_logger(), "[DEBUG] setVelocitySetpoint called: v_ned = [%.2f, %.2f, %.2f], yaw_rate = %.2f", velocity_ned.x(), velocity_ned.y(), velocity_ned.z(), yaw_rate);
    #endif
    Eigen::Vector3f acceleration_ned_m_s2 = Eigen::Vector3f::Zero(); // No acceleration
    float yaw_ned_rad = _vehicle_local_position->heading(); // Keep current heading

    _traj_setpoint->update(velocity_ned, acceleration_ned_m_s2, yaw_ned_rad, yaw_rate);
  }

private:
  rclcpp::Node & _node;

  #ifdef ROM_DEBUG
    bool debug_done_step1 = false; 
    bool debug_done_step2 = false;
  #endif

  enum class State
  {
    SettlingAtStart = 0,
    FollowRoad
  } _state;

  // --- LINE FOLLOWING PARAMETERS --- ဒါတွေကို class member အနေနဲ့ ပြောင်းသင့်ပါတယ်။
  float _kp_yaw = 0.2f;              // radian to velocity gain constant, rad to rad/s
  float _angle_tolerance = 0.2f;     // angle tolerance 11.46 degrees
  float _filtered_angle_error = 0.0f;
  float _alpha_angle = 0.2f;

  float _kp_centroid = 2.0f;         // ဘေးတိုက် အမြန်နှုန်း gain constant
  float _max_lateral_speed = 2.5f;  // ဘေးတိုက် အမြန်နှုန်း (m/s)
  float _prev_lateral_vel = 0.0f;
  float _alpha_vel = 0.3f;

  float _forward_speed = 3.0f;
  float _tracking_altitude = 15.0f;   

  // ALTITUDE CONTROL
  float desire_agl_altitude = 12.0f; // 12 meter

  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _road_line_sub;
  sensor_msgs::msg::Image::SharedPtr _latest_road_line_img;
  
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _traj_setpoint;

  void roadLineCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    _latest_road_line_img = msg;
  }
};


class ModeExecutorTest : public px4_ros2::ModeExecutorBase
{
public:
  ModeExecutorTest(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode) : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}, owned_mode), _node(node)
  {
  }

  enum class State
  {
    Reset,
    TakingOff,
    RomDynamicsMode,
    RTL,
    WaitUntilDisarmed,
  };

  void onActivate() override
  {
    runState(State::TakingOff, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    ROM_UNUSED(reason);
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) 
    {
      RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
      return;
    }

    RCLCPP_DEBUG(_node.get_logger(), "Executing state %i", (int)state);

    switch (state) {
      case State::Reset:
        break;

      case State::TakingOff:
        takeoff([this](px4_ros2::Result result) {runState(State::RomDynamicsMode, result);});
        break;

      case State::RomDynamicsMode:
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            runState(State::RTL, result);
          });
        break;

      case State::RTL:
        RCLCPP_INFO(_node.get_logger(), "Debug1 : Executing RTL");
        rtl([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
         RCLCPP_INFO(_node.get_logger(), "Debug2 : Executing RTL");
        break;

      case State::WaitUntilDisarmed:
         RCLCPP_INFO(_node.get_logger(), "Debug3 : Executing WaitUntilDisarmed");
        waitUntilDisarmed(
          [this](px4_ros2::Result result) {
            RCLCPP_INFO(_node.get_logger(), "All states complete (%s)", resultToString(result));
          });
        break;
    }
  }

private:
  rclcpp::Node & _node;
};
