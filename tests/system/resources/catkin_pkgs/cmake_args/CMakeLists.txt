cmake_minimum_required(VERSION 2.8.12)
project(cmake_args)

find_package(catkin REQUIRED)

catkin_package()

if(NOT VAR1)
  message(SEND_ERROR "VAR1 NOT DEFINED!")
endif()

if(NOT VAR2)
  message(SEND_ERROR "VAR2 NOT DEFINED!")
endif()

if(NOT VAR3)
  message(SEND_ERROR "VAR3 NOT DEFINED!")
endif()

