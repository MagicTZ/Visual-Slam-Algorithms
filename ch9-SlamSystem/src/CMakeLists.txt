cmake_minimum_required(VERSION 2.8)

project(MyBA_EIGEN)

set(CMAKE_BUILD_TYPE DEBUG)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIRS})
else()
    message(STATUS "Eigen not found")
endif()

add_library(common_lib BAL.cpp BAL.h common.h)

add_executable(BA_test main.cpp MyBA.cpp MyBA.h common.cpp)
target_link_libraries(BA_test Eigen3::Eigen common_lib)

