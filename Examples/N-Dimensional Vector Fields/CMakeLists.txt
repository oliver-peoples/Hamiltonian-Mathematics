cmake_minimum_required(VERSION 3.19)

project(n-dimensional_vector_fields)

find_package( OpenCV REQUIRED )
include_directories(~/Hamiltonian-Mathematics)

add_executable(n-dimensional_vector_fields source.cpp)

target_link_libraries(n-dimensional_vector_fields ${OpenCV_LIBS} pthread)