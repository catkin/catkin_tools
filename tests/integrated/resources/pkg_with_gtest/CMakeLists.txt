cmake_minimum_required(VERSION 2.8.3)
project(pkg_with_gtest)
find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_library(GMOCK_LIBRARY REQUIRED NAMES gmock)

  catkin_add_gtest(test_zero test_zero.cpp)
  target_link_libraries(test_zero
    ${GMOCK_LIBRARY})
endif()
