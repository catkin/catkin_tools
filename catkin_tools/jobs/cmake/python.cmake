# the CMake variable PYTHON_INSTALL_DIR has the same value as the Python function catkin.builder.get_python_install_dir()

set(PYTHON_VERSION "$ENV{ROS_PYTHON_VERSION}" CACHE STRING "Specify specific Python version to use (2 or 3)")
if("${PYTHON_VERSION}" STREQUAL "")
  message(STATUS "ROS_PYTHON_VERSION not set, using default")
endif()

find_package(Python${PYTHON_VERSION} COMPONENTS Interpreter)

if("${PYTHON_VERSION}" STREQUAL "3")
  set(_MAJOR ${Python3_VERSION_MAJOR})
  set(_MINOR ${Python3_VERSION_MINOR})
  set(_EXECUTABLE ${Python3_EXECUTABLE})
elseif("${PYTHON_VERSION}" STREQUAL "2")
  set(_MAJOR ${Python2_VERSION_MAJOR})
  set(_MINOR ${Python2_VERSION_MINOR})
  set(_EXECUTABLE ${Python2_EXECUTABLE})
else()
  set(_MAJOR ${Python_VERSION_MAJOR})
  set(_MINOR ${Python_VERSION_MINOR})
  set(_EXECUTABLE ${Python_EXECUTABLE})
endif()

message(STATUS "Using PYTHON_EXECUTABLE: ${_EXECUTABLE}")

set(_PYTHON_PATH_VERSION_SUFFIX "${_MAJOR}.${_MINOR}")

set(enable_setuptools_deb_layout OFF)
if(EXISTS "/etc/debian_version")
  set(enable_setuptools_deb_layout ON)
endif()
option(SETUPTOOLS_DEB_LAYOUT "Enable debian style python package layout" ${enable_setuptools_deb_layout})

if(SETUPTOOLS_DEB_LAYOUT)
  message(STATUS "Using Debian Python package layout")
  set(PYTHON_PACKAGES_DIR dist-packages)
  set(SETUPTOOLS_ARG_EXTRA "--install-layout=deb")
  # use major version only when installing 3.x with debian layout
  if("${_MAJOR}" STREQUAL "3")
    set(_PYTHON_PATH_VERSION_SUFFIX "${_MAJOR}")
  endif()
else()
  message(STATUS "Using default Python package layout")
  set(PYTHON_PACKAGES_DIR site-packages)
  # setuptools is fussy about windows paths, make sure the install prefix is in native format
  file(TO_NATIVE_PATH "${CMAKE_INSTALL_PREFIX}" SETUPTOOLS_INSTALL_PREFIX)
endif()

if(NOT WIN32)
  set(PYTHON_INSTALL_DIR lib/python${_PYTHON_PATH_VERSION_SUFFIX}/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
else()
  # Windows setuptools installs to lib/site-packages not lib/python2.7/site-packages
  set(PYTHON_INSTALL_DIR lib/${PYTHON_PACKAGES_DIR}
    CACHE INTERNAL "This needs to be in PYTHONPATH when 'setup.py install' is called.  And it needs to match.  But setuptools won't tell us where it will install things.")
endif()
