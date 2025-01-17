cmake_minimum_required(VERSION 3.5)
project(module_5_assignment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(angles REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${angles_INCLUDE_DIRS}
)

# Add executable
add_executable(lqr_node src/lqr_node.cpp src/lqr_lib.cpp) # Include lqr_lib.cpp

# Link libraries
ament_target_dependencies(lqr_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  angles
)

# Install executable
install(TARGETS lqr_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()