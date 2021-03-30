cmake_minimum_required(VERSION 3.0.2)
project(hakuba)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  rostime
  laser_geometry
  tf
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

file(GLOB_RECURSE PROJECT_HEADERS src/*.h)

add_executable(time_sync
  "src/time_sync.cpp"
  "src/beego_controller.cpp"
  ${PROJECT_HEADERS}
)
target_link_libraries(time_sync
  ${catkin_LIBRARIES}
)

add_executable(transform
  "src/transform.cpp"
  "src/beego_controller.cpp"
  ${PROJECT_HEADERS}
)
target_link_libraries(transform
  ${catkin_LIBRARIES}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_hakuba.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
