#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <algorithm>

  // --- LINE FOLLOWING PARAMETERS ---
  // Parameter များကို class member အနေနဲ့ ပြောင်းသင့်ပါတယ်။
  // float kp_yaw = 0.2f; // Global variable
  // float angle_tolerance = 0.2f; // Global variable
  // ...

#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

static const std::string kName = "ROM FollowRoad";

using namespace px4_ros2::literals; // NOLINT

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

    // PARAMETERS Initialization (better as class members or from ROS parameters)
    // For now, moving them inside constructor to avoid global variables.
    // Ideally, these should be loaded via ROS parameters for easy tuning.
    kp_yaw = 0.2f; // Initial value. You might need to change sign.
    angle_tolerance = 0.2f;
    filtered_angle_error = 0.0f;
    alpha_angle = 0.2f;

    kp_centroid = 2.0f; // Initial value. You might need to change sign.
    max_lateral_speed = 2.5f;
    prev_lateral_vel = 0.0f;
    alpha_vel = 0.3f;

    forward_speed = 3.0f;
  }

  void onActivate() override {    _state = State::SettlingAtStart;    }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    switch (_state) {
      case State::SettlingAtStart: {
          // Wait for road line image before starting
          if (_latest_road_line_img) {
            takeoff(_meter_height);
            _state = State::FollowRoad;
          }
        }
        break;

      case State::FollowRoad: {
          // image ကို အသုံးပြုပြီး velocity-based road following လုပ်ပါမယ်။
          if (!_latest_road_line_img) {
            // image မရှိရင် zero velocity ပဲထားမယ်။
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            RCLCPP_WARN(_node.get_logger(), "No road line image received, hovering."); // Debug message
            break;
          }

          // Convert ROS2 Image to OpenCV (assume BGR8 encoding)
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
          float center_x = W / 2.0f;

          // Threshold red channel (BGR)
          cv::Mat mask;
          // You might need to adjust this range based on your simulation's exact red color.
          // Example: if road is brighter red, increase max values.
          cv::inRange(img, cv::Scalar(0, 0, 150), cv::Scalar(50, 50, 255), mask); 
          
          // Debugging: Show the mask image to verify red line detection
          // cv::imshow("Mask", mask);
          // cv::waitKey(1);

          // Find nonzero mask pixels
          std::vector<cv::Point> pts;
          cv::findNonZero(mask, pts);
          if (pts.empty()) {
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            RCLCPP_WARN(_node.get_logger(), "No red pixels found, hovering."); // Debug message
            break;
          }

          // Fit line to red pixels
          cv::Vec4f line_params;
          cv::fitLine(pts, line_params, cv::DIST_L2, 0, 0.01, 0.01);
          float vx_fit = line_params[0], vy_fit = line_params[1];

          // Ensure the line vector consistently points "downwards" in the image.
          // If vy_fit is positive, it means the vector is pointing downwards (or mostly downwards).
          // If vy_fit is negative, it means the vector is pointing upwards, so flip it.
          if (vy_fit < 0) {
              vx_fit = -vx_fit;
              vy_fit = -vy_fit;
          }

          // Compute line angle
          float angle_line = atan2f(vy_fit, vx_fit);
          // Now, angle_line should be in (-PI, 0] range if line is generally downwards.
          // e.g., -PI/2 for perfectly vertical.
          
          float desired_angle = -M_PI / 2.0f; // Target: straight vertical line in image
          float raw_angle_error = angle_line - desired_angle;
          // Normalize angle_error to [-PI, PI)
          while (raw_angle_error > M_PI) raw_angle_error -= 2 * M_PI;
          while (raw_angle_error < -M_PI) raw_angle_error += 2 * M_PI;

          // EMA filter on angle_error
          filtered_angle_error = alpha_angle * raw_angle_error + (1.0f - alpha_angle) * filtered_angle_error;
          float angle_error = filtered_angle_error;

          // Centroid calculation
          cv::Moments M = cv::moments(mask, true);
          float centroid_x = (M.m00 == 0) ? center_x : M.m10 / M.m00;
          float error_x = centroid_x - center_x; // Positive if road is right of center
          float error_x_norm = error_x / (W / 2.0f);

          // Raw lateral velocity
          // If error_x is positive (road is right), we need to move right (positive body_right velocity).
          // So kp_centroid should be positive if a positive error_x_norm leads to a positive body_right.
          // IF YOUR DRONE DRIFTS LEFT, TRY CHANGING THE SIGN OF kp_centroid HERE:
          float raw_lateral_vel = kp_centroid * error_x_norm * max_lateral_speed;
          
          // EMA filter on lateral velocity
          float lateral_vel = alpha_vel * raw_lateral_vel + (1.0f - alpha_vel) * prev_lateral_vel;
          prev_lateral_vel = lateral_vel;

          // Body-frame commands
          float body_forward = 0.0f;
          float body_right = 0.0f;
          float body_yawrate = 0.0f;

          // --- CONTROL LOGIC ---
          // Yaw control:
          // If angle_error is positive (road is turning left), we need to yaw left (negative yawrate).
          // If angle_error is negative (road is turning right), we need to yaw right (positive yawrate).
          // So, for a positive angle_error (road turning left, need to turn left), 
          // we need a negative body_yawrate. This means -kp_yaw * angle_error is correct if kp_yaw > 0.
          // IF YOUR DRONE DRIFTS LEFT DUE TO YAW, TRY CHANGING THE SIGN OF kp_yaw:
          body_yawrate = -kp_yaw * angle_error; 

          if (fabs(angle_error) > angle_tolerance) {
            body_forward = 0.0f; // Stop forward movement if angle error is large
            body_right = 0.0f; // Stop lateral movement if angle error is large
            // Only yaw to align
          } else {
            body_forward = forward_speed;
            body_right = lateral_vel;
            // Yaw correction continues
          }

          // Convert body-frame (forward, right) to NED (PX4 expects NED)
          float psi = _vehicle_local_position->heading();
          float u = body_forward;
          float v = body_right;
          float v_north =  u * cosf(psi) - v * sinf(psi);
          float v_east  =  u * sinf(psi) + v * cosf(psi);

          setVelocitySetpoint(Eigen::Vector3f{v_north, v_east, 0.0f}, body_yawrate);

          // Debugging: Print control values
          RCLCPP_INFO(_node.get_logger(), 
                      "Angle Error: %.2f, Yaw Rate: %.2f | Centroid X Error: %.2f, Lateral Vel: %.2f | Vel N: %.2f, E: %.2f",
                      angle_error, body_yawrate, error_x, lateral_vel, v_north, v_east);
        }
        break;
    }
  }

  // Set velocity setpoint (NED frame) using PX4 TrajectorySetpoint
  void setVelocitySetpoint(const Eigen::Vector3f & velocity_ned, float yaw_rate)
  {
    Eigen::Vector3f acceleration_ned_m_s2 = Eigen::Vector3f::Zero(); // No acceleration
    float yaw_ned_rad = _vehicle_local_position->heading(); // Keep current heading

    _traj_setpoint->update(velocity_ned, acceleration_ned_m_s2, yaw_ned_rad, yaw_rate);
  }

private:
  rclcpp::Node & _node;
  float _meter_height = 7.0f;
  enum class State
  {
    SettlingAtStart = 0,
    FollowRoad
  } _state;

  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;

  // Road line image subscriber and storage
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _road_line_sub;
  sensor_msgs::msg::Image::SharedPtr _latest_road_line_img;

  // Trajectory setpoint publisher
  std::shared_ptr<px4_ros2::TrajectorySetpointType> _traj_setpoint;

  // PARAMETERS (Moved to class members)
  float kp_yaw;
  float angle_tolerance;
  float filtered_angle_error;
  float alpha_angle;

  float kp_centroid;
  float max_lateral_speed;
  float prev_lateral_vel;
  float alpha_vel;

  float forward_speed;


  void roadLineCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Store the latest image for processing
    _latest_road_line_img = msg;
  }

  void takeoff(float _meter_height)
  {
    // Get current position
    Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
    // Set target position with desired altitude (NED: negative down)
    Eigen::Vector3f target_pos = current_pos;
    target_pos.z() = -_meter_height; // PX4 NED: z is down, so negative for up

    // Zero velocity and acceleration for takeoff
    Eigen::Vector3f velocity_ned = Eigen::Vector3f::Zero();
    Eigen::Vector3f acceleration_ned = Eigen::Vector3f::Zero();
    float yaw_ned = _vehicle_local_position->heading();
    float yaw_rate = 0.0f;

    _traj_setpoint->update(target_pos, velocity_ned, yaw_ned, yaw_rate);
  }
};