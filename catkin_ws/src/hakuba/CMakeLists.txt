cmake_minimum_required(VERSION 3.0.2)
project(hakuba)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  rostime
  laser_geometry
  tf
  ecl_threads
  ecl_utilities
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hakuba
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  "/usr/include/eigen3"
)

file(GLOB_RECURSE PROJECT_SOURCES src/*.cpp)
file(GLOB_RECURSE PROJECT_HEADERS src/*.h)

add_executable(time_sync
  ${PROJECT_SOURCES}
  ${PROJECT_HEADERS}
  src/db/table_iter.cpp
)
target_link_libraries(time_sync
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS scripts/test.py
  DESTINATION  ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
        ${TEST_SOURCES}
        src/db/table_iter.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test  ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
