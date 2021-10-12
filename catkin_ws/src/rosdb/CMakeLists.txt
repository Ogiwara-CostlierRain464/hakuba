cmake_minimum_required(VERSION 3.0.2)
project(rosdb)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rostime
  laser_geometry
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosdb
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)

add_executable(ros_db
  ${PROJECT_SOURCES}
  ${PROJECT_HEADERS}
)

target_link_libraries(ros_db
  ${catkin_LIBRARIES}
)

file(GLOB_RECURSE TEST_SOURCES test/*.cpp)

list(REMOVE_ITEM PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

catkin_add_gtest(${PROJECT_NAME}-test
  ${TEST_SOURCES}
  ${PROJECT_HEADERS}
  ${PROJECT_SOURCES}
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test  ${catkin_LIBRARIES})
endif()

