cmake_minimum_required(VERSION 2.8.11)
project(md_drive_ros)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ")
set(CMAKE_BUILD_TYPE Release)

find_package ( catkin REQUIRED COMPONENTS
  roscpp dynamic_reconfigure sensor_msgs cmake_modules tf tf2_ros
)

# Dynamic reconfigure config
generate_dynamic_reconfigure_options( cfg/DriveController.cfg )

catkin_package (
  INCLUDE_DIRS include
  LIBRARIES DriveControllerNode
  CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs tf tf2_ros
)

include_directories ( 
	include 
	${catkin_INCLUDE_DIRS} 
	/usr/include/hidapi 
)

add_library(DriveControllerNode
  src/drive_controller_node.cpp src/motor_controller_api.cpp)
add_dependencies(DriveControllerNode ${PROJECT_NAME}_gencfg)
target_link_libraries(DriveControllerNode ${catkin_LIBRARIES} udev hidapi-hidraw )

add_executable(drive_controller src/main.cpp)
target_link_libraries(drive_controller DriveControllerNode ${catkin_LIBRARIES})
