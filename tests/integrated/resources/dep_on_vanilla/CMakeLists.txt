cmake_minimum_required(VERSION 2.8.3)
project(dep_on_vanilla)

find_package(catkin REQUIRED)

catkin_package()

find_file(VANILLA_TXT vanilla.txt PATHS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_SHARE_DESTINATION})

if(VANILLA_TXT STREQUAL VANILLA_TXT-NOTFOUND)
  message(FATAL_ERROR "Could not find vanilla.txt")
endif()
