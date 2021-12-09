cmake_minimum_required(VERSION 2.8.12)
project(cmake_pkg)

add_executable(vanilla vanilla.cpp)

install(TARGETS vanilla
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

