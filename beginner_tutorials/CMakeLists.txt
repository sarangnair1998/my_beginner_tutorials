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
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Catch2 2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the srv file to the build
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TogglePublishingService.srv"
)

# Link the typesupport library to executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Add executables
add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp std_msgs std_srvs tf2_ros geometry_msgs)
target_link_libraries(publisher_node ${cpp_typesupport_target})

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp std_msgs)

add_executable(test_talker tests/test_talker.cpp)
ament_target_dependencies(test_talker rclcpp std_msgs tf2_ros Catch2)

# Install targets
install(TARGETS
  publisher_node
  subscriber_node
  test_talker
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

  # Register the test using a launch file (Catch2 Integration Test)
  add_test(NAME integration_test COMMAND
    ros2 launch ${PROJECT_NAME} publisher_test_launch.py
  )
endif()

ament_package()

# Include directories for generated code
target_include_directories(publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
