cmake_minimum_required(VERSION 2.6)
project(DrawTrajectory)

set(CMAKE_CXX_STANDARD 11)
include_directories("usr/include/eighen3")
find_package(Pangolin)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
#set(Sophus_DIR "/home/magictz/Projects/slam/slambook2/3rdparty/Sophus/sophus/")
include_directories("/home/magictz/Projects/slam/slambook2/3rdparty/Sophus/")

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
#set(CPACK_PROJECT_NAME ${PROJECT_NAME})
#set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
#include(CPack)
