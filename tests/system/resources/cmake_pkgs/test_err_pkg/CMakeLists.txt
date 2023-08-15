cmake_minimum_required(VERSION 3.10)
project(test_err_pkg)

include(CTest)
add_executable(MyTest test.cpp)
add_test(NAME MyTest COMMAND MyTest)
enable_testing()

install(TARGETS MyTest
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
