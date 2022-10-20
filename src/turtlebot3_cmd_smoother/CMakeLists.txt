cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_cmd_smoother)

find_package(catkin REQUIRED 
    COMPONENTS
    geometry_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

