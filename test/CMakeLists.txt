add_executable(test1_indexer test1.cpp)
add_executable(test2_locker test2.cpp)
add_executable(test3_pathfind test3.cpp)
add_executable(test4_maze_algorithms test4.cpp)

target_link_libraries(test1_indexer PUBLIC brainmap)
target_link_libraries(test2_locker PUBLIC brainmap)
target_link_libraries(test3_pathfind PUBLIC brainmap)
target_link_libraries(test4_maze_algorithms PUBLIC brainmap)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(GTK4 QUIET IMPORTED_TARGET gtk4)
    if(GTK4_FOUND)
        add_executable(guitest guitest.cpp)
        target_link_libraries(guitest PUBLIC brainmap PkgConfig::GTK4)
    endif()
endif()

add_test(NAME Test1 COMMAND test1_indexer)
add_test(NAME Test2 COMMAND test2_locker)
add_test(NAME Test3 COMMAND test3_pathfind)
add_test(NAME Test4 COMMAND test4_maze_algorithms)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mazes
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
