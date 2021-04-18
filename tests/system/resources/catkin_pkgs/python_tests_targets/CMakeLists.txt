cmake_minimum_required(VERSION 2.8.12)
project(python_tests_targets)
find_package(catkin REQUIRED)

catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test_good.py)
  catkin_add_nosetests(test_bad.py)
endif()
