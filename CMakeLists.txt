cmake_minimum_required(VERSION 3.10)

project(LibAcross VERSION 1.0)

add_library(across)

target_include_directories(across PUBLIC include/)

target_sources(across PRIVATE src/across.cpp include/across.hpp)
