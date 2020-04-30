Cheat Sheet
===========

This is a non-exhaustive list of some common and useful invocations of the ``catkin`` command.
All of the commands which do not explicitly specify a workspace path (with ``--workspace``) are assumed to be run from within a directory contained by the target workspace.
For thorough documentation, please see the chapters on each verb.

Initializing Workspaces
^^^^^^^^^^^^^^^^^^^^^^^

Initialize a workspace with a default layout (``src``/``build``/``devel``) in the *current* directory:
  - ``catkin init``
  - ``catkin init --workspace .``
  - ``catkin config --init``
  - ``mkdir src && catkin build``

... with a default layout in a *different* directory:
  - ``catkin init --workspace /tmp/path/to/my_catkin_ws``

... which explicitly extends another workspace:
  - ``catkin config --init --extend /opt/ros/indigo``

Initialize a workspace with a **source space** called ``other_src``:
  - ``catkin config --init --source-space other_src``

... or a workspace with **build**, **devel**, and **install space** ending with the suffix ``_alternate``:
  - ``catkin config --init --space-suffix _alternate``

Configuring Workspaces
^^^^^^^^^^^^^^^^^^^^^^

View the current configuration:
  - ``catkin config``

Setting and unsetting CMake options:
  - ``catkin config --cmake-args -DENABLE_CORBA=ON -DCORBA_IMPLEMENTATION=OMNIORB``
  - ``catkin config --no-cmake-args``

Toggle installing to the specified **install space**:
  - ``catkin config --install``

Building Packages
^^^^^^^^^^^^^^^^^

Build all the packages:
  - ``catkin build``

... one at a time, with additional debug output:
  - ``catkin build -p 1``

... and force CMake to re-configure for each one:
  - ``catkin build --force-cmake``

Build a specific package and its dependencies:
  - ``catkin build my_package``

... or ignore its dependencies:
  - ``catkin build my_package --no-deps``

Build the package containing the current working directory:
  - ``catkin build --this``

... but don't rebuild its dependencies:
  - ``catkin build --this --no-deps``

Build packages with additional CMake args:
  - ``catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug``

... and save them to be used for the next build:
  - ``catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug``

Build all packages in a given directory:
  - ``catkin build $(catkin list -u -d /path/to/folder)``

... or in the current folder:
  - ``catkin build $(catkin list -u -d .)``


Cleaning Build Products
^^^^^^^^^^^^^^^^^^^^^^^

Blow away the build, devel, and install spaces (if they exist):
  - ``catkin clean``

... or just the **build space**:
  - ``catkin clean --build``

... or just clean a single package:
  - ``catkin clean PKGNAME``

... or just delete the build directories for packages which have been disabled or removed:
  - ``catkin clean --orphans``

Controlling Color Display
^^^^^^^^^^^^^^^^^^^^^^^^^

Disable colors when building in a shell that doesn't support it (like IDEs):
  - ``catkin --no-color build``

... or enable it for shells that don't know they support it:
  - ``catkin --force-color build``

Profile Cookbook
^^^^^^^^^^^^^^^^

Create "Debug" and "Release" profiles and then build them in independent build and devel spaces:
  .. code-block:: bash

    catkin config --profile debug -x _debug --cmake-args -DCMAKE_BUILD_TYPE=Debug
    catkin config --profile release -x _release --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build --profile debug
    catkin build --profile release

Quickly build a package from scratch to make sure all of its dependencies are satisfied, then clean it:
  .. code-block:: bash

    catkin config --profile my_pkg -x _my_pkg_test
    catkin build --profile my_pkg my_pkg
    catkin clean --profile my_pkg --all

Manipulating Workspace Chaining
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Change from implicit to explicit chaining:
  .. code-block:: bash

    catkin clean
    catkin config --extend /opt/ros/indigo

Change from explicit to implicit chaining:
  .. code-block:: bash

    catkin clean
    catkin config --no-extend

Building With Other Job Servers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build with ``distcc``:
  .. code-block:: bash

     CC="distcc gcc" CXX="distcc g++" catkin build -p$(distcc -j) -j$(distcc -j) --no-jobserver
     
Changing Package's Build Type
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Set the build type to ``cmake`` in the ``package.xml`` file's ``<export/>`` section:
  .. code-block:: xml
    
    <export>
      <build_type>cmake</build_type>
    </export>
