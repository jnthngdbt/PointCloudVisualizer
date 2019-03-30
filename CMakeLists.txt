cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(VisualizerTest)
find_package(PCL 1.9 REQUIRED COMPONENTS common features filters io search visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES src/*.h src/*.cpp src/*.hpp)
add_executable(VisualizerTest ${SOURCES})

target_link_libraries(VisualizerTest ${PCL_LIBRARIES})
