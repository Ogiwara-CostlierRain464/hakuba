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
  topic_tools
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

file(GLOB_RECURSE RECORDER_SOURCES src/*.cpp)
file(GLOB_RECURSE PLAYER_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)

list(REMOVE_ITEM RECORDER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/play.cpp)
list(REMOVE_ITEM PLAYER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/record.cpp)

add_executable(record
  ${RECORDER_SOURCES}
  ${PROJECT_HEADERS}
)

add_executable(play
  ${PLAYER_SOURCES}
  ${PROJECT_HEADERS}
)

target_link_libraries(record
  ${catkin_LIBRARIES}
)

target_link_libraries(play
  ${catkin_LIBRARIES}
)

file(GLOB_RECURSE TEST_SOURCES test/*.cpp)

list(REMOVE_ITEM RECORDER_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/record.cpp)

catkin_add_gtest(${PROJECT_NAME}-test
  ${TEST_SOURCES}
  ${RECORDER_SOURCES}
  ${PROJECT_HEADERS}
)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test  ${catkin_LIBRARIES})
endif()

