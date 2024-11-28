cmake_minimum_required(VERSION 3.0)
project(groot)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

message("PATH" ${CMAKE_MODULE_PATH})
find_package(PCL 1.13 REQUIRED COMPONENTS common io geometry kdtree search features surface visualization filters gpu_containers gpu_octree)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(realsense2)
include_directories(${CMAKE_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(examples)
add_subdirectory(tests)





