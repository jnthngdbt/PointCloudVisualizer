cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(PointCloudVisualizer)

find_package(PCL 1.9 REQUIRED COMPONENTS common features filters io search visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB VisualizerFiles 
  src/Visualizer.h 
  src/Visualizer.cpp 
  src/Visualizer.hpp 
  src/VisualizerData.h 
  src/VisualizerData.cpp 
  src/VisualizerData.hpp)
file(GLOB VisualizerAppFiles src/VisualizerApp.cpp)
file(GLOB VisualizerTestFiles src/VisualizerTest.cpp)
file(GLOB ProjectFiles src/stdafx.h src/stdafx.cpp src/targetver.h)

add_executable(VisualizerTest ${ProjectFiles} ${VisualizerFiles} ${VisualizerTestFiles})
target_link_libraries(VisualizerTest ${PCL_LIBRARIES})

add_executable(VisualizerApp ${ProjectFiles} ${VisualizerFiles} ${VisualizerAppFiles})
target_link_libraries(VisualizerApp ${PCL_LIBRARIES})