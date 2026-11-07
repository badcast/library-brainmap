cmake_minimum_required(VERSION 3.12)

# PROJECT

file(READ "VERSION" FILE_PROJECT_VERSION)

string(STRIP "${FILE_PROJECT_VERSION}" FILE_PROJECT_VERSION)

project(library-brainmap VERSION ${FILE_PROJECT_VERSION})

option(BUILD_TESTING "Build testing for check module" ON)
option(BRAIN_SANITIZE "Build the tests with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# TARGET

add_library(brainmap INTERFACE)

target_include_directories(
  brainmap INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)

set_target_properties(brainmap PROPERTIES VERSION ${FILE_PROJECT_VERSION})

# TESTS

# warnings and sanitizers are for the tests only: they are not forced on the users of the library
add_library(brainmap_dev_flags INTERFACE)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(brainmap_dev_flags INTERFACE -Wall -Wextra)
  if(BRAIN_SANITIZE)
    target_compile_options(brainmap_dev_flags INTERFACE -fsanitize=address,undefined -fno-omit-frame-pointer
                                                       -fno-sanitize-recover=undefined)
    target_link_options(brainmap_dev_flags INTERFACE -fsanitize=address,undefined)
  endif()
endif()

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(3rdParty)
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

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/brainmap.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/brainmap.pc @ONLY)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/brainmap.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# CPACK (the variables must be set before include(CPack))

set(CPACK_PACKAGE_NAME "library-brainmap")
set(CPACK_PACKAGE_VERSION "${FILE_PROJECT_VERSION}")
if(WIN32)
  set(CPACK_GENERATOR "NSIS")
else()
  set(CPACK_GENERATOR "TGZ")
endif()

include(CPack)
