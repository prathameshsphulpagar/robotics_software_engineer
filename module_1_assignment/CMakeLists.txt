cmake_minimum_required(VERSION 3.8)
project(module_1_assignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_executable(task_1 src/task_1.cpp)
target_link_libraries(task_1)

add_executable(task_2 src/task_2.cpp)
target_link_libraries(task_2)

ament_package()


