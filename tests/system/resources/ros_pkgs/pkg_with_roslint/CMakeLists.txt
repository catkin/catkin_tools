cmake_minimum_required(VERSION 3.10)
project(pkg_with_roslint)
find_package(catkin REQUIRED COMPONENTS roslint)
catkin_package()

roslint_cpp(main.cpp)
roslint_add_test()
