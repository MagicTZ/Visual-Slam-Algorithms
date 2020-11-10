cmake_minimum_required(VERSION 2.8)

project(OpticalFlow)

set(CMAKE_CXX_FLAGS  "-std=c++11 -O2")

set(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
else()
    message(STATUS "OpenCV not found")
endif(OpenCV_FOUND)

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIRS})
else()
    message(STATUS "Eigen3 not found")
endif(Eigen3_FOUND)


## Sophus Library
find_package(Sophus REQUIRED)
if(Sophus_FOUND)
    include_directories(${Sophus_INCLUDE_DIRS})
else()
    message(STATUS "Sophus not found")
endif(Sophus_FOUND)

## Pangolin Library
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp draw_pointcloud.h)
target_link_libraries(optical_flow ${OpenCV_LIBS} Eigen3::Eigen)

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method Eigen3::Eigen ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})