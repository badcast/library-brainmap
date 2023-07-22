cmake_minimum_required(VERSION 3.10)

project(LibAcross VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include/)
include_directories(test/)

enable_testing()

add_executable(test_indexer test/test1.cpp)
add_executable(test_point_neight test/test2.cpp)
add_executable(test_locker test/test3.cpp)

add_test(NAME test1 COMMAND test_indexer)
add_test(NAME test2 COMMAND test_point_neight)
add_test(NAME test3 COMMAND test_locker)
