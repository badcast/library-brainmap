add_executable(test1_indexer test1.cpp)
add_executable(test2_locker test2.cpp)
add_executable(test3_pathfind test3.cpp)
add_executable(test4_maze_algorithms test4.cpp)
add_executable(test5_bidirectional test5_bidirectional.cpp)

target_link_libraries(test1_indexer PUBLIC brainmap)
target_link_libraries(test2_locker PUBLIC brainmap)
target_link_libraries(test3_pathfind PUBLIC brainmap)
target_link_libraries(test4_maze_algorithms PUBLIC brainmap)
target_link_libraries(test5_bidirectional PUBLIC brainmap)

if(TARGET imgui)
    add_executable(guitest guitest.cpp)
    target_link_libraries(guitest PUBLIC brainmap imgui)
endif()

add_test(NAME Test1 COMMAND test1_indexer)
add_test(NAME Test2 COMMAND test2_locker)
add_test(NAME Test3 COMMAND test3_pathfind)
add_test(NAME Test4 COMMAND test4_maze_algorithms)
add_test(NAME Test5 COMMAND test5_bidirectional)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/mazes
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
