project(disparity)

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
else(Pangolin_FOUND)
    message(STATUS "Pangolin not found")
endif(Pangolin_FOUND)

add_executable(disparity disparity.cpp)
# 注意链接Pangolin库的时候，名字和OpenCV不一样
target_link_libraries(disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBS})