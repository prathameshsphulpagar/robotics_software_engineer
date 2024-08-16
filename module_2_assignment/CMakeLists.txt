cmake_minimum_required(VERSION 3.8)
project(module_2_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)

add_executable(task2 src/task2.cpp)
ament_target_dependencies(task2 rclcpp geometry_msgs)

add_executable(task1 src/task1.cpp)
ament_target_dependencies(task1 rclcpp geometry_msgs)

add_executable(task3 src/task3.cpp)
ament_target_dependencies(task3 rclcpp geometry_msgs)

add_executable(task4 src/task4.cpp)
ament_target_dependencies(task4 rclcpp geometry_msgs)

install(DIRECTORY
  launch
  src
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  task2
  task1
  task3
  task4
  DESTINATION lib/${PROJECT_NAME})

ament_package()
