cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Add executables
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs std_srvs)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

# Install targets
install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # The following line skips the linter which checks for copyrights
  # Comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # Comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
