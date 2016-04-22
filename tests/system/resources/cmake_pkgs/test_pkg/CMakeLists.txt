cmake_minimum_required (VERSION 2.6)
project(test_package_pkg)

enable_testing()

add_executable(vanilla_test_app vanilla.cpp)
find_library(LIBVANILLA vanilla)
target_link_libraries(vanilla_test_app ${LIBVANILLA})

add_test(vanilla_test vanilla_test_app)

install(TARGETS vanilla_test_app
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/static)

