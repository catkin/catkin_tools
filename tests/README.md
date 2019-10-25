Testing
=======

The `catkin_tools` test harness includes the following types of tests,
organized into different directories:

* **unit** -- API tests for the `catkin_tools` python interface
* **system** -- Tests which not only test integrated parts of `catkin_tools`
  but the interaction with other, external projects like catkin_pkg and catkin.

## Running Tests

All tests can be run from the root of the repository.

First, make sure the required test dependencies are installed:

*Ubuntu* -- `sudo apt-get install cmake libgtest-dev build-essential python3-setuptools python3-pip`

*OS X* -- `pip install setuptools`

*Both Platforms*

```
pip install argparse catkin-pkg distribute PyYAML psutil
pip install nose coverage flake8 mock empy --upgrade
pip install git+https://github.com/osrf/osrf_pycommon.git
```

Second, build the Catkin CMake tool:

```
git clone https://github.com/ros/catkin.git /tmp/catkin_source -b indigo-devel --depth 1
mkdir /tmp/catkin_source/build
pushd /tmp/catkin_source/build
cmake .. && make
source devel/setup.bash
popd
```

Finally, install `catkin_tools`:

```
python setup.py develop
```

To run all tests and view the output, run the following from the repository root:

```
python setup.py nosetests -s
```
