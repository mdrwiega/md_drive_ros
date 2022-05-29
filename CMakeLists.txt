cmake_minimum_required(VERSION 3.1)
project(md_drive_ros)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} ")
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs cmake_modules tf tf2_ros
)

find_library(MD_DRIVE_API NAMES md_drive_api)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DriveControllerNode
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf tf2_ros
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(DriveControllerNode
  src/drive_controller_node.cpp
  src/odometry.cpp
)

target_link_libraries(DriveControllerNode ${catkin_LIBRARIES} ${MD_DRIVE_API})

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} DriveControllerNode ${catkin_LIBRARIES})
