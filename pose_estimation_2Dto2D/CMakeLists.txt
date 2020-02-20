cmake_minimum_required(VERSION 2.6)
project(pose)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

add_executable(pose main.cpp)

target_link_libraries(pose ${OpenCV_LIBS})
install(TARGETS pose RUNTIME DESTINATION bin)


