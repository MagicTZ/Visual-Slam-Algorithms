cmake_minimum_required(VERSION 2.3)

project(useEigen)

include_directories("/usr/include/eigen3")

add_executable(useEigen eigenMatrix.cpp)

add_executable(useGeometry useGeometry.cpp)

add_executable(coordinateTransform coordinateTransform.cpp)