cmake_minimum_required(VERSION 2.8.3)
project(vrep_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  snake_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_control_for_vrep
  CATKIN_DEPENDS roscpp rospy std_msgs snake_msgs
  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

## Declare a C++ library
 add_library(${PROJECT_NAME}
#   src/robot_control_for_vrep.cpp
   src/vrep.cpp

 )


 add_executable(${PROJECT_NAME}_node 
			src/main.cpp
			src/vrep.cpp
			src/control_robot_for_vrep.cpp
  			src/extApi.c
  			src/extApiPlatform.c

		)

 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
