cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

# Declare executables
add_executable(pre_approach_node src/pre_approach.cpp)
ament_target_dependencies(pre_approach_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros)

add_executable(pre_approach_node_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_node_v2 rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros std_srvs)
rosidl_target_interfaces(pre_approach_node_v2 ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros std_srvs)
rosidl_target_interfaces(approach_service_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executables
install(TARGETS
  pre_approach_node
  pre_approach_node_v2
  approach_service_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Package configuration
ament_package()
