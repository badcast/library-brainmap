add_executable(test1_indexer test1.cpp)
add_executable(test2_locker test2.cpp)
add_executable(test3_pathfind  test3.cpp)

target_link_libraries(test1_indexer PUBLIC libacross)
target_link_libraries(test2_locker PUBLIC libacross)
target_link_libraries(test3_pathfind PUBLIC libacross)

add_test(NAME Test1 COMMAND test1_indexer)
add_test(NAME Test2 COMMAND test2_locker)
add_test(NAME Test3 COMMAND test3_pathfind)
