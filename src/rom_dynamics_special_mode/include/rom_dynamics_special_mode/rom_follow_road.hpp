  // --- LINE FOLLOWING PARAMETERS ---
  float kp_yaw = 0.2f;
  float angle_tolerance = 0.2f;
  float filtered_angle_error = 0.0f;
  float alpha_angle = 0.2f;

  float kp_centroid = 2.0f;
  float max_lateral_speed = 2.5f;
  float prev_lateral_vel = 0.0f;
  float alpha_vel = 0.3f;

  float forward_speed = 3.0f;

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

static const std::string kName = "ROM FollowRoad";

using namespace px4_ros2::literals; // NOLINT

class RoadFollowMode : public px4_ros2::ModeBase
{
public:
  explicit RoadFollowMode(rclcpp::Node & node) : ModeBase(node, kName)
  {
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Subscribe to /road_line topic (sensor_msgs::msg::Image)
    _road_line_sub = node.create_subscription<sensor_msgs::msg::Image>(
      "/road_line",
      10,
      std::bind(&RoadFollowMode::roadLineCallback, this, std::placeholders::_1)
    );

    // Publisher for trajectory setpoint (velocity control)
    _traj_pub = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
  }

  void onActivate() override
  {
    _state = State::SettlingAtStart;
  }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    switch (_state) {
      case State::SettlingAtStart: {
          // Wait for road line image before starting
          if (_latest_road_line_img) {
            _state = State::FollowRoad;
          }
        }
        break;

      case State::FollowRoad: {
          // Velocity-based road following using image
          if (!_latest_road_line_img) {
            // No image, hover (zero velocity)
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }

          // Convert ROS2 Image to OpenCV (assume BGR8 encoding)
          cv_bridge::CvImagePtr cv_ptr;
          try {
            cv_ptr = cv_bridge::toCvCopy(_latest_road_line_img, "bgr8");
          } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }
          cv::Mat img = cv_ptr->image;
          int H = img.rows, W = img.cols;
          float center_x = W / 2.0f;

          // Threshold red channel (BGR)
          cv::Mat mask;
          cv::inRange(img, cv::Scalar(0, 0, 151), cv::Scalar(49, 49, 255), mask);

          // Find nonzero mask pixels
          std::vector<cv::Point> pts;
          cv::findNonZero(mask, pts);
          if (pts.empty()) {
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }

          // Fit line to red pixels
          cv::Vec4f line_params;
          cv::fitLine(pts, line_params, cv::DIST_L2, 0, 0.01, 0.01);
          float vx_fit = line_params[0], vy_fit = line_params[1];

          // Compute line angle
          float angle_line = atan2f(vy_fit, vx_fit);
          if (angle_line > 0) angle_line -= M_PI;
          float desired_angle = -M_PI / 2.0f;
          float raw_angle_error = angle_line - desired_angle;
          while (raw_angle_error > M_PI) raw_angle_error -= 2 * M_PI;
          while (raw_angle_error < -M_PI) raw_angle_error += 2 * M_PI;

          // EMA filter on angle_error
          filtered_angle_error = alpha_angle * raw_angle_error + (1.0f - alpha_angle) * filtered_angle_error;
          float angle_error = filtered_angle_error;

          // Centroid calculation
          cv::Moments M = cv::moments(mask, true);
          float centroid_x = (M.m00 == 0) ? center_x : M.m10 / M.m00;
          float error_x = centroid_x - center_x;
          float error_x_norm = error_x / (W / 2.0f);

          // Raw lateral velocity
          float raw_lateral_vel = -kp_centroid * error_x_norm * max_lateral_speed;
          // EMA filter on lateral velocity
          float lateral_vel = alpha_vel * raw_lateral_vel + (1.0f - alpha_vel) * prev_lateral_vel;
          prev_lateral_vel = lateral_vel;

          // Body-frame commands
          float body_forward = 0.0f;
          float body_right = 0.0f;
          float body_yawrate = 0.0f;

          if (fabs(angle_error) > angle_tolerance) {
            body_forward = 0.0f;
            body_right = 0.0f;
            body_yawrate = -kp_yaw * angle_error;
          } else {
            body_forward = forward_speed;
            body_right = lateral_vel;
            body_yawrate = -kp_yaw * angle_error;
          }

          // Convert body-frame (forward, right) to NED (PX4 expects NED)
          float psi = _vehicle_local_position->heading();
          float u = body_forward;
          float v = body_right;
          float v_north =  u * cosf(psi) - v * sinf(psi);
          float v_east  =  u * sinf(psi) + v * cosf(psi);

          setVelocitySetpoint(Eigen::Vector3f{v_north, v_east, 0.0f}, body_yawrate);
        }
        break;
    }
  }

private:
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
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _traj_pub;

  void roadLineCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Store the latest image for processing
    _latest_road_line_img = msg;
    // You can add image processing logic here as needed
  }

  // Set velocity setpoint (NED frame) using PX4 TrajectorySetpoint
  void setVelocitySetpoint(const Eigen::Vector3f & velocity_ned, float yaw_rate)
  {
    px4_msgs::msg::TrajectorySetpoint msg;
    msg.velocity[0] = velocity_ned.x(); // North
    msg.velocity[1] = velocity_ned.y(); // East
    msg.velocity[2] = velocity_ned.z(); // Down
    msg.yaw = _vehicle_local_position->heading(); // Current heading (rad)
    msg.yaw_speed = yaw_rate; // Yaw rate (rad/s)
    _traj_pub->publish(msg);
  }
};
