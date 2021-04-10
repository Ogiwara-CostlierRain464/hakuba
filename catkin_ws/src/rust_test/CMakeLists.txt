cmake_minimum_required(VERSION 3.0)
set (ENV{PROJECT_NAME} rust_test)

project($ENV{PROJECT_NAME})

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        )
catkin_package(
        CATKIN_DEPENDS std_msgs
)


add_custom_target($ENV{PROJECT_NAME}
        ALL
        COMMAND cargo build --release -p $ENV{PROJECT_NAME}
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/cargo/release/$ENV{PROJECT_NAME} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$ENV{PROJECT_NAME}
        COMMENT "Building my Rust library"
        )

unset (ENV{PROJECT_NAME})