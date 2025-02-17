cmake_minimum_required(VERSION 3.8)
project(cbr_2025_fase3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#find dependencies
find_package(ament_cmake REQUIRED)
find_package(fsm REQUIRED)
find_package(cbr_drone_lib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(vision_msgs REQUIRED)

include_directories(include)

add_executable(fase3
  src/fase3.cpp
)

ament_target_dependencies(fase3
  rclcpp
  fsm
  cbr_drone_lib
  Eigen3
  vision_msgs
)

add_executable(fase3_middle
  src/fase3_middle.cpp
)

ament_target_dependencies(fase3_middle
  rclcpp
  fsm
  cbr_drone_lib
  Eigen3
  vision_msgs
)

install(
  TARGETS
    fase3
    fase3_middle
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
