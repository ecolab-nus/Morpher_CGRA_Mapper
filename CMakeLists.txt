cmake_minimum_required(VERSION 3.1)
find_package(Threads REQUIRED)
set(CMAKE_CXX_FLAGS -pthread)
add_subdirectory(src)