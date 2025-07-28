cmake_minimum_required(VERSION 3.8)
project(rom_dynamics_special_mode)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(go_triangle src/rom_gotriangle_mode.cpp)
target_link_libraries(go_triangle PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

add_executable(takeoff_gt_land_executor src/rom_takeoff_gotriangle_land_mode_executor.cpp)
target_link_libraries(takeoff_gt_land_executor PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

add_executable(go_triangle_land src/rom_gotriangle_land_mode.cpp)
target_link_libraries(go_triangle_land PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

add_executable(follow_road src/autonomous_follow_road.cpp)
target_link_libraries(follow_road PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp ${OpenCV_LIBS} cv_bridge::cv_bridge)

add_executable(test_setpoint src/test_setpoint.cpp)
target_link_libraries(test_setpoint PUBLIC Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp ${OpenCV_LIBS} cv_bridge::cv_bridge)
install(TARGETS test_setpoint DESTINATION lib/${PROJECT_NAME})

install(TARGETS
        go_triangle
        go_triangle_land
        takeoff_gt_land_executor
        follow_road
        test_setpoint
        DESTINATION lib/${PROJECT_NAME})

ament_package()
