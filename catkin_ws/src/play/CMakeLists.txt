cmake_minimum_required(VERSION 3.0.2)
project(play)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        rostime
        laser_geometry
        tf
        ecl_threads
        ecl_utilities
        move_base_msgs
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

add_executable(play
        ${PROJECT_SOURCES}
        ${PROJECT_HEADERS}
        )
target_link_libraries(play
        ${catkin_LIBRARIES}
        )

#############
## Testing ##
#############