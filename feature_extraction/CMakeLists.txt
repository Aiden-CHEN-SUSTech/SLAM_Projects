cmake_minimum_required(VERSION 2.6)
project(feature_extraction)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})
add_executable(feature_extraction main.cpp)

target_link_libraries(feature_extraction  ${OpenCV_LIBS})
