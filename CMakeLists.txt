cmake_minimum_required(VERSION 3.8)
project(omegabot)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Curses REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(
  arduino_controller
  src/arduino_controller/cmd_vel_subscriber.cpp
  src/arduino_controller/connect.hpp
  src/arduino_controller/connect.cpp
  src/arduino/config.h
  )

ament_target_dependencies(arduino_controller rclcpp geometry_msgs)

install(DIRECTORY
  config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  arduino_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
