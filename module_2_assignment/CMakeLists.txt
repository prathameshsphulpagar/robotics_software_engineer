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

add_executable(drive src/drive.cpp)
ament_target_dependencies(drive rclcpp geometry_msgs)

add_executable(test1 src/test.cpp)
ament_target_dependencies(test1 rclcpp geometry_msgs)

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
  drive
  test1
  task3
  task4
  DESTINATION lib/${PROJECT_NAME})

ament_package()
