cmake_minimum_required(VERSION 3.15)
project(CalibrationTest)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP)


find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(CalibrationTest main.cpp)
target_link_libraries(CalibrationTest PUBLIC ${PCL_LIBRARIES})
if(OpenMP_CXX_FOUND)
    target_link_libraries(CalibrationTest PUBLIC OpenMP::OpenMP_CXX)
endif()