cmake_minimum_required(VERSION 3.5.1)
project(communication)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(
  CATKIN_PACKAGE_DEPENDENCIES 
    grid_map_core
    grid_map_ros
    grid_map_msgs
    roscpp
    std_msgs
    tf
    tf_conversions
)

find_package(catkin REQUIRED 
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    kdl_parser
    robot_state_publisher
)

find_package(lcm)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_library
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

###########
# Library #
###########

add_library(${PROJECT_NAME}_library
  src/communication_node.cpp
)

target_link_libraries(${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
  ${LCM_NAMESPACE}lcm
)

##############
# Executable #
##############

add_executable(${PROJECT_NAME}_node
  src/communication_node
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_library
)