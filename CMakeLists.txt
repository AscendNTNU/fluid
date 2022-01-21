cmake_minimum_required(VERSION 3.5)
project(fluid)

Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(ascend_msgs REQUIRED)

add_executable(fluid src/nodes/main.cpp)
add_executable(base_link_publisher src/nodes/base_link_publisher.cpp)
ament_target_dependencies(main rclcpp std_msgs std_srvs geometry_msgs nav_msgs mavros_msgs ascend_msgs)

install(TARGETS
  main
  base_link_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()