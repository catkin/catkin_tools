cmake_minimum_required(VERSION 3.10)
project(cmake_pkg)

add_executable(vanilla vanilla.cpp)

install(TARGETS vanilla
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
