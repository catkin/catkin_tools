cmake_minimum_required(VERSION 2.8.3)
project(pkg_with_cmake_warning)

find_package(catkin REQUIRED)
catkin_package()

message(WARNING "This package warns you in the cmake stage")
