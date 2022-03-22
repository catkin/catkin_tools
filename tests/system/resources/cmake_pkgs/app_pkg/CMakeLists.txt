cmake_minimum_required(VERSION 2.8.12)
project(app_pkg)

find_package(lib_pkg REQUIRED)

include_directories(${lib_pkg_INCLUDE_DIRS})

add_executable(vanilla_app vanilla.cpp)
target_link_libraries(vanilla_app vanilla)

install(TARGETS vanilla_app
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)
