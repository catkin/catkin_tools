cmake_minimum_required(VERSION 3.10)
project(cmake_err)
find_package(catkin REQUIRED)
catkin_package()
message(SEND_ERROR "This package sends an error from cmake.")
