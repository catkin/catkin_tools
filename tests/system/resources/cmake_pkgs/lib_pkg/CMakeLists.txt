cmake_minimum_required(VERSION 2.8.12)
project(lib_pkg)

include_directories(${CMAKE_SOURCE_DIR})
add_library(vanilla SHARED vanilla.cpp)

INSTALL(FILES vanilla.h
  DESTINATION include
  )

install(TARGETS vanilla
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

