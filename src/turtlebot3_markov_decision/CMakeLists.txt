cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_markov_decision)

find_package(catkin REQUIRED 
    COMPONENTS
    visualization_msgs
    nav_msgs
    geometry_msgs
    std_msgs
)

catkin_package(CATKIN_DEPENDS nav_msgs)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

