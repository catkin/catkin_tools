set(PYTHON_VERSION "$ENV{ROS_PYTHON_VERSION}" CACHE STRING "Specify specific Python version to use ('major.minor' or 'major')")
find_package(PythonInterp ${PYTHON_VERSION} REQUIRED QUIET)
message("${PYTHON_VERSION_MAJOR};${PYTHON_VERSION_MINOR}")
