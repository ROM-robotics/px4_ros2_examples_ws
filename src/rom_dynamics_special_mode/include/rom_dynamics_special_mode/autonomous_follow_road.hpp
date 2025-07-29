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

  // --- LINE FOLLOWING PARAMETERS --- ဒါတွေကို class member အနေနဲ့ ပြောင်းသင့်ပါတယ်။
  float kp_yaw = 0.2f;              // radian to velocity gain constant, rad to rad/s
  float angle_tolerance = 0.2f;     // angle tolerance 11.46 degrees
  float filtered_angle_error = 0.0f;
  float alpha_angle = 0.2f;

  float kp_centroid = 2.0f;         // ဘေးတိုက် အမြန်နှုန်း gain constant
  float max_lateral_speed = 2.5f;  // ဘေးတိုက် အမြန်နှုန်း (m/s)
  float prev_lateral_vel = 0.0f;
  float alpha_vel = 0.3f;

  float forward_speed = 3.0f;
#define ROM_DEBUG 1
#define ROM_UNUSED(x) (void)(x)
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
  }

  void onActivate() override {    _state = State::SettlingAtStart;    }

  void onDeactivate() override {}

  void updateSetpoint(float dt_s) override
  {
    switch (_state) {
      case State::SettlingAtStart: {
          // Wait for road line image before starting
          if (_latest_road_line_img) {
            //takeoff(_meter_height);
            _state = State::FollowRoad;
          }
        }
        break;

      case State::FollowRoad: {
          // image ကို အသုံးပြုပြီး velocity-based road following လုပ်ပါမယ်။
          if (!_latest_road_line_img) {
            // image မရှိရင် zero velocity ပဲထားမယ်။
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            RCLCPP_WARN(_node.get_logger(), "No road line image received, hovering.");
            break;
          }

          // ROS2 image ကို computer vision image ပြောင်းမယ်။ (assume BGR8 encoding)
          // ပြောင်းလို့ မရရင် velocity setpoint ကို zero လုပ်ပါမယ်။
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
            // RCLCPP_INFO(_node.get_logger(), "Altitude : %.3f", _vehicle_local_position->positionNed().z()) ;
          #endif
          
          // အမဲနောက်ခံ အနီ line ပါတဲ့ image ထဲက Threshold red channel (BGR) အနီသည် white 255 ဖြစ်လာပြီး အနီမဟုတ်တာက black 0 ဖြစ်လာမယ်။
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
          
          // Find nonzero mask pixels
          // အနီ pixels တစ်ခုစီရဲ့ x,y တွေပါတဲ့ vector pts ကို ဆောက်တယ်။
          std::vector<cv::Point> pts;
          cv::findNonZero(mask, pts);
          // အနီ pixels မရှိရင် zero velocity ပဲထားမယ်။
          if (pts.empty()) {
            setVelocitySetpoint(Eigen::Vector3f::Zero(), 0.0f);
            break;
          }

          // အနီ points များအတွက် fit line ဆွဲပြီး အဲ့ဒီ fit line ရဲ့ vx နဲ့  vy vector များကိုရယူမယ်။
          cv::Vec4f line_params; // (x1, y1, x2, y2) format or (vx, vy, x0, y0)
          cv::fitLine(pts, line_params, cv::DIST_L2, 0, 0.01, 0.01);
          float vx_fit = line_params[0], vy_fit = line_params[1];
          
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] Fit line params: vx: %.4f, vy: %.4f", vx_fit, vy_fit);
          #endif

          // Compute theta angle
          // atan2 က ၁၈၀  နဲ့ -၁၈၀ ကြားရှိမယ်။ 
          float theta_angle = atan2f(vy_fit, vx_fit);
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] theta_angle_deg %.4f", theta_angle * 57.2958);
          #endif

          // ဒီနှစ်လိုင်းက တွက်ကြည့်ရင် ဥပမာ  theta_angle က 30 degree ဖြစ်ရင် desired_angle က -90 အမြဲတမ်း constant ဖြစ်ပြီး raw anglee error က -60 ဖြစ်ပါတယ်။
          //if (theta_angle > 0) theta_angle -= M_PI;
          float desired_angle = M_PI / 2.0f;

          // အဖြစ်ယူဆမယ်ဆိုရင် vertical line ကနေ error သည် -60 degree အနေနဲ့ယူဆမယ်။
          float raw_angle_error = fabs(theta_angle) - desired_angle;
          if(theta_angle < 0) {
            // theta_angle က negative ဖြစ်ရင် angle error ကို -1 လုပ်မယ်။
            raw_angle_error *= -1.0f;
          }
          #ifdef ROM_DEBUG1
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP2 ] raw_angle_error %.4f", raw_angle_error * 57.2958);
          #endif

          while (raw_angle_error > M_PI) raw_angle_error -= 2 * M_PI;
          while (raw_angle_error < -M_PI) raw_angle_error += 2 * M_PI;

          // EMA filter on angle_error, aplha_angle က 0.2 ဖြစ်တဲ့အတွက် previous filtered_angle_error ကို 80% အထိ သုံးမယ်။
          filtered_angle_error = alpha_angle * raw_angle_error + (1.0f - alpha_angle) * filtered_angle_error;
          float angle_error = filtered_angle_error;
          // ============================================================== angle_error တွက်ပြီးပြီ။
          // Centroid calculation
          cv::Moments M = cv::moments(mask, true);

          // Detect မိရင် centroid_x ကိုတွက်မယ်။
          float centroid_x = (M.m00 == 0) ? center_x : M.m10 / M.m00;
          // image ရဲ့ center နဲ့ object ရဲ့ center ကြားက အကွာအဝေး
          float error_x = centroid_x - center_x;
          // error သည် -320, 320 ကြားရှိတာမို့ 320 နဲ့ စားပြီး -1 နဲ့ 1 ကြားရအောင် Normalize လုပ်ထားတယ်။
          float error_x_norm = error_x / (W / 2.0f);

          // error မရှိရင် speed က 0
          // error -1 ဖြစ်ရင် -5, error 1 ဖြစ်ရင် 5 ဖြစ်မယ်။
          float raw_lateral_vel = kp_centroid * error_x_norm * max_lateral_speed;
          // EMA နဲ့ noise ကို စစ်ထုတ်မယ်။
          float lateral_vel = alpha_vel * raw_lateral_vel + (1.0f - alpha_vel) * prev_lateral_vel;
          prev_lateral_vel = lateral_vel;

          // Body-frame commands
          float body_forward = 0.0f;
          float body_right = 0.0f;
          float body_yawrate = 0.0f;

          // ================================================================ angle error ပြန်သုံးမယ်။
          // angle_error က 20 degree ထက်ပိုရင် body_forward, body_right ကို 0 လုပ်ပြီး yawrate ကို ပဲတွက်မယ်
          if (fabs(angle_error) > angle_tolerance) 
          {
            body_forward = 0.0f;
            body_right = 0.0f;
            body_yawrate = kp_yaw * angle_error; // 0.2 * 0.3491 rad(20 degrees) = 0.06982 rad/s
          } 
          // angle_error က 20 degree ထက်နည်းရင် အကုန်တွက်မယ်။
          else {
            body_forward = forward_speed;
            body_right = lateral_vel;
            body_yawrate = kp_yaw * angle_error;
          }
          #ifdef ROM_DEBUG
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP3 ] angle_err(deg): %.4f, kp_yaw: %.4f, yaw_rate(rad/s): %.4f", angle_error * 57.2958, kp_yaw, body_yawrate);
          #endif

          // Convert body-frame (forward, right) to NED (PX4 expects NED)
          float ψ = _vehicle_local_position->heading(); // ψ က drone ရဲ့ ့ heading, yaw angle
          float u = body_forward;
          float v = body_right;
          float v_north =  u * cosf(ψ) - v * sinf(ψ);
          float v_east  =  u * sinf(ψ) + v * cosf(ψ);

          // float v_north =  body_right;
          // float v_east  =  body_forward;

          setVelocitySetpoint(Eigen::Vector3f{v_north, v_east, 0.0f}, body_yawrate);
          #ifdef ROM_DEBUG
              RCLCPP_INFO(_node.get_logger(), "[ ROM DEBUG STEP3 ] v_north(m/s): %.4f, v_east(m/s): %.4f", v_north, body_right);
          #endif

        }
        break;
    }
  }

  // Set velocity setpoint (NED frame) using PX4 TrajectorySetpoint
  void setVelocitySetpoint(const Eigen::Vector3f & velocity_ned, float yaw_rate)
  {
    // px4_msgs::msg::TrajectorySetpoint msg;
    // msg.velocity[0] = velocity_ned.x(); // North
    // msg.velocity[1] = velocity_ned.y(); // East
    // msg.velocity[2] = velocity_ned.z(); // Down
    // msg.yaw = _vehicle_local_position->heading(); // Current heading (rad)
    // msg.yawspeed = yaw_rate; // Yaw rate (rad/s)
    // _traj_pub->publish(msg);
    
    Eigen::Vector3f acceleration_ned_m_s2 = Eigen::Vector3f::Zero(); // No acceleration
    float yaw_ned_rad = _vehicle_local_position->heading(); // Keep current heading

    _traj_setpoint->update(velocity_ned, acceleration_ned_m_s2, yaw_ned_rad, yaw_rate);
  }

private:
  rclcpp::Node & _node;
  float _meter_height = 15.0f;

  #ifdef ROM_DEBUG
    bool debug_done_step1 = false; 
    bool debug_done_step2 = false;
  #endif

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
  //rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _traj_pub;

  void roadLineCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Store the latest image for processing
    _latest_road_line_img = msg;
    // You can add image processing logic here as needed
  }

//   void takeoff(float _meter_height)
// {
//     // Get current position
//     Eigen::Vector3f current_pos = _vehicle_local_position->positionNed();
//     px4_msgs::msg::TrajectorySetpoint setpoint;
//     // Set target position with desired altitude (NED: negative down)
//     setpoint.position[0] = current_pos.x(); // North
//     setpoint.position[1] = current_pos.y(); // East
//     setpoint.position[2] = _meter_height;  // Down (negative up)
//     // Zero velocity for takeoff
//     setpoint.velocity[0] = 0.0f;
//     setpoint.velocity[1] = 0.0f;
//     setpoint.velocity[2] = 0.5f;
//     setpoint.yaw = _vehicle_local_position->heading();
//     setpoint.yawspeed = 0.0f;

//     _traj_setpoint->update(setpoint);

//     #ifdef ROM_DEBUG
//       RCLCPP_INFO(_node.get_logger(), "Takeoff to : %.1f meter",_meter_height);
//     #endif
// }
};
