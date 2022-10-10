cmake_minimum_required(VERSION 3.0.2)
project(turtle3_gridmap_to_pcl)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  tf
)

# Package
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
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
