cmake_minimum_required(VERSION 3.10)

project(LibAcross VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include/)

message(STATUS "Building executer ...")
add_executable(test include/across.hpp include/across_impl.hpp
                    src/main-test.cpp)
