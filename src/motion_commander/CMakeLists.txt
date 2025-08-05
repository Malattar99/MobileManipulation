cmake_minimum_required(VERSION 3.5)
project(motion_commander)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Executable
add_executable(motion_generator_node src/motion_generator_node.cpp)
add_executable(tracking_generator_node src/tracking_generator_node.cpp)

ament_target_dependencies(motion_generator_node
  rclcpp
  tf2_ros
  visualization_msgs
  tf2_geometry_msgs 
  trajectory_msgs
  std_srvs
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
)

ament_target_dependencies(tracking_generator_node 
  rclcpp 
  tf2_ros
  visualization_msgs
  tf2_geometry_msgs 
  trajectory_msgs
  std_srvs
  sensor_msgs
  urdf 
  kdl_parser
  orocos_kdl
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS motion_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tracking_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
