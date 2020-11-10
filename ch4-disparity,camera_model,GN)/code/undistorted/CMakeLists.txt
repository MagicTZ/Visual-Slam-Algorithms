project(undistorted)

add_executable(undistort_image undistort_image.cpp)
target_link_libraries(undistort_image ${OpenCV_LIBS})