#include "CustomMode.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>

// Constants
static const std::string kModeName = "CustomMode";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

CustomMode::CustomMode(rclcpp::Node& node): ModeBase(node, kModeName), _node(node)
{
    // Initialize the trajectory setpoint and odometry local position
    _trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    // Subscribe to vehicle_land_detected
    _vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
                         "/fmu/out/vehicle_land_detected", rclcpp::QoS(1).best_effort(),
                         std::bind(&CustomMode::vehicleLandDetectedCallback, this, std::placeholders::_1));

    // Declare the trajectory type parameter with a default value of "circle"
    _node.declare_parameter("trajectory_type", "circle");
    _node.get_parameter("trajectory_type", _trajectory_type);
}

void CustomMode::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
    _land_detected = msg->landed;
}

void CustomMode::onActivate()
{
    generateWaypoints();
    switchToState(State::Execute);
}

void CustomMode::onDeactivate()
{
    RCLCPP_INFO(_node.get_logger(), "Deactivating CustomMode");
}

void CustomMode::updateSetpoint(float dt_s)
{
    switch (_state) {
    case State::Execute: {
        // Get the next waypoint
        Eigen::Vector3f target_position = _waypoints[_waypoint_index];

        // Construct a trajectory setpoint message
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = _node.now().nanoseconds() / 1000;
        setpoint.position = {target_position.x(), target_position.y(),
                     target_position.z()
                    };
        setpoint.velocity = {NAN, NAN, NAN};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.jerk = {NAN, NAN, NAN};
        setpoint.yaw = NAN;
        setpoint.yawspeed = NAN;

        _trajectory_setpoint->update(setpoint);

        if (positionReached(target_position)) {
            // Move to the next waypoint
            _waypoint_index++;

            // If we have Executeed all waypoints, go ReturnToHome
            if (_waypoint_index >= static_cast<int>(_waypoints.size())) {
                switchToState(State::ReturnToHome);
            }
        }

        break;
    }

    case State::ReturnToHome: {
        // Set the target position to the X,Y position where the drone was armed
        Eigen::Vector3f target_position = {
            0.0f, 0.0f, _vehicle_local_position->positionNed().z()
        };
        // Construct a trajectory setpoint message
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = _node.now().nanoseconds() / 1000;
        setpoint.position = {target_position.x(), target_position.y(),
                     target_position.z()
                    };
        setpoint.velocity = {NAN, NAN, NAN};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.jerk = {NAN, NAN, NAN};
        setpoint.yaw = NAN;
        setpoint.yawspeed = NAN;

        _trajectory_setpoint->update(setpoint);

        if (positionReached(target_position)) {
            switchToState(State::Descend);
        }

        break;
    }

    case State::Descend: {
        // Construct a trajectory setpoint message
        px4_msgs::msg::TrajectorySetpoint setpoint;
        setpoint.timestamp = _node.now().nanoseconds() / 1000;
        setpoint.position = {0.0, 0.0, NAN};
        setpoint.velocity = {NAN, NAN, 0.35};
        setpoint.acceleration = {NAN, NAN, NAN};
        setpoint.jerk = {NAN, NAN, NAN};
        setpoint.yaw = NAN;
        setpoint.yawspeed = NAN;

        _trajectory_setpoint->update(setpoint);

        if (_land_detected) {
            switchToState(State::Finished);
        }

        break;
    }

    // Finished state is the state where the mode is completed
    case State::Finished: {
        ModeBase::completed(px4_ros2::Result::Success);
        break;
    }
    } // end switch/case
}

// Helper function to add points along a line segment
void addLineSegment(std::vector<Eigen::Vector3f>& waypoints,
                    const Eigen::Vector3f& start_point,
                    const Eigen::Vector3f& end_point,
                    int num_segments)
{
    for (int i = 0; i <= num_segments; ++i) {
        float t = static_cast<float>(i) / num_segments;
        float x = start_point.x() + t * (end_point.x() - start_point.x());
        float y = start_point.y() + t * (end_point.y() - start_point.y());
        float z = start_point.z() + t * (end_point.z() - start_point.z());
        waypoints.push_back(Eigen::Vector3f(x, y, z));
    }
}

void CustomMode::generateWaypoints()
{
    // Set the start position
    double start_y = 0.0; // Y-center for the letters
    double current_z = _vehicle_local_position->positionNed().z();

    // Set the maximum radius and number of points
    double max_radius = 3.0; // This will be the approximate height/width of letters
    int points = 16; // Used for general curves like circle/spiral, and as a base for letter segments

    // Create a vector to store the waypoints
    std::vector<Eigen::Vector3f> waypoints;

    // Trajectory type spiral
    if (_trajectory_type == "spiral") {

        // Parameters for the spiral pattern
        double radius = 0.0;

        // Generate spiral waypoints
        // The spiral waypoints are generated in the NED frame
        for (int point = 0; point < points + 1; ++point) {
            double angle = 2.0 * M_PI * point / points;
            double x = 0.0 + radius * cos(angle); // Use 0.0 for spiral center
            double y = 0.0 + radius * sin(angle); // Use 0.0 for spiral center
            double z = current_z;
            // Push the waypoints to the vector
            waypoints.push_back(Eigen::Vector3f(x, y, z));
            // Increase the radius
            radius += max_radius / points;
        }

    }

    // Trajectory type figure 8
    else if (_trajectory_type == "figure_8") {
        // Parameters for the figure 8 pattern
        double angle_increment = 2.0 * M_PI / points;

        // Generate figure 8 waypoints
        // The figure 8 waypoints are generated in the NED frame
        for (int point = 0; point < points; ++point) {
            double angle = angle_increment * point;

            // Create the figure-8 pattern
            double x = 0.0 + max_radius * sin(angle); // Use 0.0 for figure 8 center
            double y = 0.0 + max_radius * sin(2 * angle); // Use 0.0 for figure 8 center
            double z = current_z; // Assuming constant z
            // Push the waypoints to the vector
            waypoints.push_back(Eigen::Vector3f(x, y, z));
        }
    }

    // Trajectory type ROM
    else if (_trajectory_type == "ROM") {
        // Parameters for drawing letters
        const double letter_height = max_radius;
        const double letter_width = max_radius * 0.7; // Adjust width for aesthetics
        const double letter_spacing = max_radius * 0.2;
        const int segment_points = 20; // More points for smoother lines/curves

        // Calculate total width to center the word "ROM"
        const double total_rom_width = (letter_width * 3) + (letter_spacing * 2);
        double current_x_offset = -total_rom_width / 2.0; // Start X to center the word

        // Vertical offset to center the letters around start_y (0.0)
        // This makes 0.0 the vertical center of the letter, not its base.
        const double vertical_center_offset = letter_height / 2.0;

        // --- Draw Letter 'R' (Block Style) ---
        double r_current_x = current_x_offset;
        double r_base_y = start_y + vertical_center_offset; // Bottom of the letter
        double r_top_y = start_y - letter_height + vertical_center_offset; // Top of the letter
        double r_mid_y = start_y - letter_height / 2.0 + vertical_center_offset; // Mid-height for the 'P' loop and diagonal start
        double r_p_width = letter_width * 0.7; // Width of the 'P' section

        // 1. Left Vertical Line (Stem)
        addLineSegment(waypoints,
                       Eigen::Vector3f(r_current_x, r_base_y, current_z),
                       Eigen::Vector3f(r_current_x, r_top_y, current_z),
                       segment_points);

        // 2. Top Horizontal Line of 'P'
        addLineSegment(waypoints,
                       Eigen::Vector3f(r_current_x, r_top_y, current_z),
                       Eigen::Vector3f(r_current_x + r_p_width, r_top_y, current_z),
                       segment_points / 2);

        // 3. Right Vertical Line of 'P'
        addLineSegment(waypoints,
                       Eigen::Vector3f(r_current_x + r_p_width, r_top_y, current_z),
                       Eigen::Vector3f(r_current_x + r_p_width, r_mid_y, current_z),
                       segment_points / 2);

        // 4. Bottom Horizontal Line of 'P' (connecting back to stem)
        addLineSegment(waypoints,
                       Eigen::Vector3f(r_current_x + r_p_width, r_mid_y, current_z),
                       Eigen::Vector3f(r_current_x, r_mid_y, current_z),
                       segment_points / 2);

        // 5. Diagonal Leg of 'R'
        // Starts from the mid-point of the stem (or where the 'P' loop ends)
        // Ends at the bottom-right.
        addLineSegment(waypoints,
                       Eigen::Vector3f(r_current_x, r_mid_y, current_z), // Start from where 'P' loop closes
                       Eigen::Vector3f(r_current_x + letter_width, r_base_y, current_z), // End at bottom-right
                       segment_points);

        current_x_offset += letter_width + letter_spacing;

        // --- Draw Letter 'O' ---
        double o_center_x = current_x_offset + letter_width / 2.0;
        double o_center_y = start_y + vertical_center_offset - letter_height / 2.0; // Center of the 'O'
        double o_radius_x = letter_width / 2.0;
        double o_radius_y = letter_height / 2.0;

        for (int i = 0; i <= segment_points; ++i) {
            double angle = 2.0 * M_PI * (static_cast<double>(i) / segment_points);
            double x = o_center_x + o_radius_x * cos(angle);
            double y = o_center_y + o_radius_y * sin(angle);
            waypoints.push_back(Eigen::Vector3f(x, y, current_z));
        }

        current_x_offset += letter_width + letter_spacing;

        // --- Draw Letter 'M' ---
        double m_current_x = current_x_offset;
        double m_base_y = start_y + vertical_center_offset;
        double m_top_y = start_y - letter_height + vertical_center_offset;
        double m_mid_y_peak = start_y - letter_height * 0.6 + vertical_center_offset; // Peak of the 'V' shape

        // 1. Left Vertical Line of 'M'
        addLineSegment(waypoints,
                       Eigen::Vector3f(m_current_x, m_base_y, current_z),
                       Eigen::Vector3f(m_current_x, m_top_y, current_z),
                       segment_points);

        // 2. First Diagonal Down (from top-left to mid-bottom)
        addLineSegment(waypoints,
                       Eigen::Vector3f(m_current_x, m_top_y, current_z),
                       Eigen::Vector3f(m_current_x + letter_width / 2.0, m_mid_y_peak, current_z),
                       segment_points);

        // 3. Second Diagonal Up (from mid-bottom to top-right)
        addLineSegment(waypoints,
                       Eigen::Vector3f(m_current_x + letter_width / 2.0, m_mid_y_peak, current_z),
                       Eigen::Vector3f(m_current_x + letter_width, m_top_y, current_z),
                       segment_points);

        // 4. Right Vertical Line of 'M'
        addLineSegment(waypoints,
                       Eigen::Vector3f(m_current_x + letter_width, m_top_y, current_z),
                       Eigen::Vector3f(m_current_x + letter_width, m_base_y, current_z),
                       segment_points);

    }

    // Trajectory type circle
    else  {
        // Parameters for the circle pattern
        double angle_increment = 2.0 * M_PI / points;

        // Generate circle waypoints
        // The circle waypoints are generated in the NED frame
        for (int point = 0; point < points; ++point) {
            double angle = angle_increment * point;
            double x = 0.0 + max_radius * cos(angle); // Use 0.0 for circle center
            double y = 0.0 + max_radius * sin(angle); // Use 0.0 for circle center
            double z = current_z;
            // Push the waypoints to the vector
            waypoints.push_back(Eigen::Vector3f(x, y, z));
        }

    }

    _waypoints = waypoints;
}

bool CustomMode::positionReached(const Eigen::Vector3f& target) const
{
    // Define the position and velocity thresholds
    static constexpr float kDeltaPosition = 0.25f;
    static constexpr float kDeltaVelocity = 0.25f;

    // Get the current position and velocity
    auto position = _vehicle_local_position->positionNed();
    auto velocity = _vehicle_local_position->velocityNed();

    const auto delta_pos = target - position;

    return (delta_pos.norm() < kDeltaPosition) &&
           (velocity.norm() < kDeltaVelocity);
}

std::string CustomMode::stateName(State state)
{
    switch (state) {
    case State::ReturnToHome:
        return "ReturnToHome";

    case State::Execute:
        return "Execute";

    case State::Descend:
        return "Descend";

    case State::Finished:
        return "Finished";

    default:
        return "Unknown";
    }
}

void CustomMode::switchToState(State state)
{
    RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
    _state = state;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<CustomMode>>(
                 kModeName, kEnableDebugOutput));
    rclcpp::shutdown();
    return 0;
}
