cmake_minimum_required(VERSION 3.5)
project(md_drive_ros)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_library(MD_DRIVE_API NAMES md_drive_api)

ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME}
  src/drive_controller_node.cpp
  src/odometry.cpp
)

target_link_libraries(${PROJECT_NAME} ${MD_DRIVE_API})

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_exe ${PROJECT_NAME})

# Unit tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

# Export package
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(${PROJECT_NAME} PUBLIC std_msgs geometry_msgs sensor_msgs tf tf2_ros)

### Install
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
