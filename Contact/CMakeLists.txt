cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Contact)
find_package(PCL 1.9 REQUIRED COMPONENTS common features io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB SOURCES *.h *.cpp)
add_executable(Contact ${SOURCES})

target_link_libraries(Contact ${PCL_LIBRARIES})
