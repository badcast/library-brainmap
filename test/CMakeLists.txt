add_executable(test1_indexer test1.cpp)
add_executable(test2_point_neight test2.cpp)
add_executable(test3_locker test3.cpp)

target_link_libraries(test1_indexer PUBLIC libacross)
target_link_libraries(test2_point_neight PUBLIC libacross)
target_link_libraries(test3_locker PUBLIC libacross)

add_test(NAME test1 COMMAND test1_indexer)
add_test(NAME test2 COMMAND test2_point_neight)
add_test(NAME test3 COMMAND test3_locker)
