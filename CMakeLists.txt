cmake_minimum_required(VERSION 3.8)
project(omegabot)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-pthread")

find_package(ament_cmake REQUIRED)
find_package(Curses REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

add_executable(arduino_controller src/App/main.cpp src/Arduino/Config.h src/App/graphics.h src/App/Connect.hpp
               src/App/Connect.cpp src/App/Exception.hpp src/App/header.h)

target_link_libraries(arduino_controller ${CURSES_LIBRARIES})

#add_executable(arduino_controller src/App/arduino_controller.cpp src/App/arduino_controller.hpp)
#ament_target_dependencies(arduino_controller rclcpp std_msgs)

add_executable(keyboard_controller src/App/keyboard_controller.cpp src/App/keyboard_controller.hpp src/Arduino/Config.h)
ament_target_dependencies(keyboard_controller rclcpp std_msgs)

target_link_libraries(keyboard_controller ${CURSES_LIBRARIES})

install(DIRECTORY
  config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  arduino_controller
  keyboard_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
