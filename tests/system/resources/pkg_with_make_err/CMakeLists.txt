cmake_minimum_required(VERSION 2.8.3)
project(pkg_with_make_err)

find_package(catkin REQUIRED)

catkin_package()

add_executable(main main.cpp)
