cmake_minimum_required(VERSION 3.0.0)
project(PA4 VERSION 0.1.0)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    #添加头文件
    include_directories(${OpenCV_INCLUDE_DIRS})
    find_package(Eigen3)
    if(Eigen3_FOUND)
        include_directories(${Eigen3_INCLUDE_DIRS})
    else()
        message(STATUS "Eigen3 not found")
    endif()
else(OpenCV_FOUND)
    message(STATUS "OpenCV not found")
endif(OpenCV_FOUND)


add_subdirectory(undistorted)
add_subdirectory(disparity)
add_subdirectory(gaussnewton)
