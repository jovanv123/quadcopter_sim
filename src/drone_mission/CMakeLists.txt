cmake_minimum_required(VERSION 3.8)
project(drone_mission LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(mission_node
  src/main.cpp
  src/mission_node.cpp
)
target_link_libraries(mission_node Eigen3::Eigen) 
target_include_directories(mission_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mission_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS mission_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
