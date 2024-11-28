cmake_minimum_required(VERSION 2.6)
project(MY_PROJECT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.9.1 REQUIRED COMPONENTS common io geometry kdtree search features surface visualization filters)
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(realsense2)

add_library(groot JpegEncoder.cpp Frame.cpp)
target_link_libraries(groot ${PCL_LIBRARIES} turbojpeg)

add_executable(server main.cpp)
target_link_libraries(server ${PCL_LIBRARIES} groot realsense2)



