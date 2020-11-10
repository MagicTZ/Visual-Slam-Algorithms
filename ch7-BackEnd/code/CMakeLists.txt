cmake_minimum_required(VERSION 2.8)

project(G2O)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(CSparse REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse g2o_types_slam3d)

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

add_library(bal_common BAL.cpp)
#add_executable(g2oBA g2oBA.cpp)
#target_link_libraries(g2oBA ${G2O_LIBS} bal_common ${OpenCV_LIBS})

add_executable(directBA directBA.cpp)
target_link_libraries(directBA ${G2O_LIBS} bal_common ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

#add_executable(directBA_2 directBA_2.cpp)
#target_link_libraries(directBA_2 ${G2O_LIBS} bal_common ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})