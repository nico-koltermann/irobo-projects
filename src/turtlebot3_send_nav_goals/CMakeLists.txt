cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_send_nav_goals)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  actionlib
  costmap_2d
  geometry_msgs
  move_base_msgs
  visualization_msgs
)

# Package
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/main.cpp
  )

target_link_libraries(
  ${PROJECT_NAME} ${catkin_LIBRARIES}
)
