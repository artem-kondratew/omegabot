cmake_minimum_required(VERSION 3.8)
project(omegabot)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-pthread")

find_package(ament_cmake REQUIRED)
find_package(Curses REQUIRED)

include_directories(${CURSES_INCLUDE_DIR})

add_executable(arduino_controller src/App/main.cpp src/Arduino/Config.h src/App/graphics.h src/App/Connect.h
               src/App/Connect.cpp src/App/str.h src/App/str.cpp src/App/Gservo.h src/App/Gservo.cpp src/App/History.h
               src/App/List.h src/App/Exception.h src/App/Vision.h src/App/Vision.cpp src/App/header.h)

target_link_libraries(arduino_controller ${CURSES_LIBRARIES})

# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})
# target_link_libraries(Hackaton ${OpenCV_LIBS})

install(DIRECTORY
  config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  arduino_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()