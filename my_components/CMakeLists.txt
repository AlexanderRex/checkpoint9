cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(attach_shelf)

include_directories(include)

add_library(pre_approach_component SHARED src/pre_approach.cpp)
target_compile_definitions(pre_approach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
   )
rclcpp_components_register_nodes(pre_approach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach_component>\n")

add_library(attach_server_component SHARED src/attach_server.cpp)
target_compile_definitions(attach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  "std_srvs"
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "attach_shelf")
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attach_server_component>\n")

add_library(attach_client_component SHARED src/attach_client.cpp)
target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  "std_srvs"
  "rclcpp"
  "rclcpp_components"
  "attach_shelf")
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:attach_client_component>\n")

install(TARGETS
  attach_server_component
  pre_approach_component
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()