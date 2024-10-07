cmake_minimum_required(VERSION 3.8)
project(module_4_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(task3 src/task3.cpp)
ament_target_dependencies(task3 rclcpp sensor_msgs geometry_msgs)

add_executable(imu_data_process src/imu_data_process.cpp)
ament_target_dependencies(imu_data_process rclcpp sensor_msgs)

install(DIRECTORY
  launch
  src
  worlds
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  # task2
  # task1
  task3
  # task4
  imu_data_process
  DESTINATION lib/${PROJECT_NAME})

ament_package()
