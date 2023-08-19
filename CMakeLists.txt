cmake_minimum_required(VERSION 3.12)

# PROJECT

file(READ "VERSION" FILE_PROJECT_VERSION)

string(STRIP "${FILE_PROJECT_VERSION}" FILE_PROJECT_VERSION)

project(library-brainmap VERSION ${FILE_PROJECT_VERSION})

option(BUILD_TESTING "Build testing for check module" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TARGET

add_library(brainmap INTERFACE)

target_include_directories(
  brainmap INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)

set_target_properties(brainmap PROPERTIES VERSION ${FILE_PROJECT_VERSION})

# TESTS

enable_testing()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# INSTALLS

include(GNUInstallDirs)

install(TARGETS brainmap EXPORT brainmapTargets)

install(
  EXPORT brainmapTargets
  FILE brainmapTargets.cmake
  NAMESPACE brainmap::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/brainmap)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/brainmap.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/brainmap.pc @ONLY)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/brainmap)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/brainmap.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
