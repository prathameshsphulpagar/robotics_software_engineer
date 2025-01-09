cmake_minimum_required(VERSION 3.8)
project(path_planning)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED on)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(EIGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/../../extern/eigen)
find_package(Eigen3 REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)

add_library(lqr_lib STATIC src/lqr_lib.cpp)
target_include_directories(lqr_lib PRIVATE ${EIGEN_INCLUDE_DIR} include)
target_link_libraries(lqr_lib Eigen3::Eigen)

add_executable(lqr_node src/lqr_node.cpp)
target_link_libraries(lqr_node lqr_lib Eigen3::Eigen).
ament_target_dependencies(lqr_node rclcpp nav_msgs geometry_msgs tf2 angles)
target_include_directories(lqr_node PRIVATE ${EIGEN_INCLUDE_DIR} include)

install(DIRECTORY
  lqr_lib
  lqr_node
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  launch 
  config
  DESTINATION lib/${PROJECT_NAME})
ament_package()
