cmake_minimum_required(VERSION 2.6)
project(learnusegeometry)

include_directories("/usr/include/eigen3")

add_executable(learnusegeometry main.cpp)

install(TARGETS learnusegeometry RUNTIME DESTINATION bin)
