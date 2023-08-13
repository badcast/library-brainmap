add_executable(test1_indexer test1.cpp)
add_executable(test2_locker test2.cpp)

target_link_libraries(test1_indexer PUBLIC libacross)
target_link_libraries(test2_locker PUBLIC libacross)

add_test(NAME test1 COMMAND test1_indexer)
add_test(NAME test2 COMMAND test2_locker)
