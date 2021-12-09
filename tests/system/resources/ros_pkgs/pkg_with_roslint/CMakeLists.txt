cmake_minimum_required(VERSION 2.8.12)
project(pkg_with_roslint)
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

roslint_cpp(main.cpp)
roslint_add_test()
