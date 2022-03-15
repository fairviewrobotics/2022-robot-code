cmake_minimum_required(VERSION 3.16)
project(high_goal_vision)

set(CMAKE_CXX_STANDARD 17)

# Enable gcc/clang warnings
if(CMAKE_CXX_COMPILER_ID MATCHES GNU OR CMAKE_CXX_COMPILER_ID MATCHES Clang)
    set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -Wno-unused-parameter")
    set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(vision_test high_goal.cpp camera_distort.cpp vision_test.cpp)
target_link_libraries(vision_test ${OpenCV_LIBS} )

add_executable(camera_calibration camera_calibration.cpp camera_distort.cpp)
target_link_libraries(camera_calibration ${OpenCV_LIBS} )