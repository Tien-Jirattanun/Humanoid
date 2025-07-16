cmake_minimum_required(VERSION 3.8)
project(hanuman_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# Include directories
include_directories(
  include
  include/hanuman_control
  include/trajectory
)

# add executable
add_executable(
  hanuman_control 
  # src/single_loop_publisher.cpp
  src/cascade_controller_publisher.cpp
  src/lib/pid_control.cpp
  src/lib/cascade_control.cpp
)

ament_target_dependencies(
  hanuman_control 
  rclcpp 
  std_msgs
  sensor_msgs
)

install(TARGETS
  hanuman_control
  DESTINATION lib/${PROJECT_NAME})

# py module
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
