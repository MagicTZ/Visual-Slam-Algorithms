project(gaussnewton)

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS} Eigen3::Eigen)