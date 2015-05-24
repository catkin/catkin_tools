cmake_minimum_required(VERSION 2.8.3)
project(pkg_with_products)
find_package(catkin REQUIRED)
catkin_package()

add_executable(main.cpp)
