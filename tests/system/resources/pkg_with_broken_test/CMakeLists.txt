cmake_minimum_required(VERSION 2.8.3)
project(pkg_with_broken_test)
find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test_bad.py)
endif()
