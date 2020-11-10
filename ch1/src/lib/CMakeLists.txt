set(LIBHELLO_SRC hello.cpp hello.h)

add_library(hello ${LIBHELLO_SRC})
add_library(hello_shared SHARED ${LIBHELLO_SRC})

add_executable(sayhello useHello.cpp)
target_link_libraries(sayhello hello_shared)

install(FILES hello.h DESTINATION include/)
install(TARGETS hello_shared DESTINATION lib/)
