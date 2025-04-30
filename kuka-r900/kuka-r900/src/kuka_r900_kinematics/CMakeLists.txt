cmake_minimum_required(VERSION 3.5)
project(kuka_r900_kinematics)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(trajectory_msgs REQUIRED)








add_executable(kdl_ik_node src/kdl_ik_node.cpp)
ament_target_dependencies(kdl_ik_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  trajectory_msgs
  kdl_parser
  orocos_kdl
  urdf
)

install(TARGETS
  kdl_ik_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(kdl_fk_node src/kdl_fk_node.cpp)
ament_target_dependencies(kdl_fk_node
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  orocos_kdl
  urdf
)
install(TARGETS 
  kdl_fk_node 
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

