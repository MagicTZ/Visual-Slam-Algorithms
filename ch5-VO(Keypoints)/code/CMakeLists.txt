cmake_minimum_required(VERSION 2.8)

project(ORB)

set(CMAKE_CXX_FLAGS  "-std=c++11 -O2")

set(CMAKE_BUILD_TYPE "Release")

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

find_package(Sophus REQUIRED)
if(Sophus_FOUND)
    include_directories(${Sophus_INCLUDE_DIRS})
else()
    message(STATUS "Sophus not found")
endif(Sophus_FOUND)

find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${OpenCV_LIBS} Eigen3::Eigen ${Sophus_LIBRARIES})

add_executable(GN_BA GN-BA.cpp)
target_link_libraries(GN_BA ${Sophus_LIBRARIES} Eigen3::Eigen)

add_executable(ICP ICP.cpp)
target_link_libraries(ICP ${Sophus_LIBRARIES} Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES})