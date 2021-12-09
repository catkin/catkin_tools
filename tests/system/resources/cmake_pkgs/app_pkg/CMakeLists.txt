cmake_minimum_required(VERSION 2.8.12)
project(app_pkg)

add_executable(vanilla_app vanilla.cpp)
find_library(LIBVANILLA vanilla)
target_link_libraries(vanilla_app ${LIBVANILLA})

install(TARGETS vanilla_app
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

