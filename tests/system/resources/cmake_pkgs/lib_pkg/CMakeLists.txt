cmake_minimum_required(VERSION 3.10)
project(lib_pkg)

# make cache variables for install destinations
include(GNUInstallDirs)

include_directories(${CMAKE_SOURCE_DIR})
add_library(vanilla SHARED vanilla.cpp)

# install the target and create export-set
install(TARGETS vanilla
  EXPORT vanillaTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# install header file
INSTALL(FILES vanilla.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# generate and install export file
install(EXPORT vanillaTargets
        FILE vanillaTargets.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib_pkg
)

# include CMakePackageConfigHelpers macro
include(CMakePackageConfigHelpers)

# create config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/lib_pkgConfig.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib_pkg
)

# install config files
install(FILES
          "${CMAKE_CURRENT_BINARY_DIR}/lib_pkgConfig.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lib_pkg
)

# generate the export targets for the build tree
export(EXPORT vanillaTargets
       FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/vanillaTargets.cmake"
)
