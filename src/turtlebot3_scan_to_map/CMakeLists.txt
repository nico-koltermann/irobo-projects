cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_scan_to_map)

find_package(catkin REQUIRED 
    COMPONENTS
    visualization_msgs
    nav_msgs
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS nav_msgs)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

