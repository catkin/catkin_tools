cmake_minimum_required(VERSION 2.8.12)
project(products_unicode)
find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CFG_EXTRAS extras.cmake)

include_directories(include)

add_library(${PROJECT_NAME}_lib lib.cpp)
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME "${PROJECT_NAME}_lïb")
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(main main.cpp)
set_target_properties(main PROPERTIES OUTPUT_NAME "maïn")
target_link_libraries(main ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_lib main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
