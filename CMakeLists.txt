cmake_minimum_required(VERSION 3.12)

#PROJECT

project(libacross VERSION 1.0.0)

option(BUILD_TESTING "Build testing for check module" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TARGET

add_library(libacross INTERFACE)

target_include_directories(libacross INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# TESTS

if(BUILD_TESTING)
    add_subdirectory(test)
endif()

# INSTALLS

include(GNUInstallDirs)

install(TARGETS libacross
    EXPORT libacrossTargets
)

install(EXPORT libacrossTargets
    FILE libacrossTargets.cmake
    NAMESPACE libacross::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/libacross
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/libacross.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libacross.pc @ONLY
)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/across
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libacross.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
