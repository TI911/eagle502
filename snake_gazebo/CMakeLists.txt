cmake_minimum_required(VERSION 2.8.3)
project(snake_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  snake_msgs
  gazebo_ros	
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snakebot_gazebo
   CATKIN_DEPENDS roscpp std_msgs snake_msgs 
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(snake_gazebo_node 
     src/snake_gazebo.cpp
     src/main.cpp
)

target_link_libraries(snake_gazebo_node
   ${catkin_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
