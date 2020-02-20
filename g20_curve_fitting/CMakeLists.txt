cmake_minimum_required(VERSION 2.6)
project(g20_curve_fitting)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_DIRS})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS} "/usr/local/include")

include_directories("/usr/local/include")

add_executable(g20_curve_fitting main.cpp)

install(TARGETS g20_curve_fitting RUNTIME DESTINATION bin)
