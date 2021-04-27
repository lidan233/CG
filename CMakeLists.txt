cmake_minimum_required(VERSION 3.14)
project(CG)

set(CMAKE_CXX_STANDARD 14)

include(
        cmake/LidanBase.cmake
        cmake/LidanBuild.cmake
)


add_executable(CG main.cpp)




