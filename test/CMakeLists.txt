cmake_minimum_required(VERSION 2.6)
project(test)
set(CMAKE_BUILD_TYPE "Debug" )
add_executable(test main.cpp)

install(TARGETS test RUNTIME DESTINATION bin)
