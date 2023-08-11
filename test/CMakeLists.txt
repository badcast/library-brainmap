include_directories(../include)

add_executable(test1_indexer test1.cpp)
add_executable(test2_point_neight test2.cpp)
add_executable(test3_locker test3.cpp)

add_test(NAME test1 COMMAND test1_indexer)
add_test(NAME test2 COMMAND test2_point_neight)
add_test(NAME test3 COMMAND test3_locker)
