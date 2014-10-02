Cheat Sheet
===========

This is a non-exhaustive list of some common and useful invocations of the ``catkin`` command.
All of the commands which do not explicitly specify a workspace path (with ``--workspace``)
are assumed to be run from within a directory contained by the target workspace. For thorough
documentation, please see the chapters on each verb.

Initializing Workspaces
^^^^^^^^^^^^^^^^^^^^^^^

Initialize a workspace with a default layout (``src``/``build``/``devel``) in the *current* directory:
  - ``catkin init``
  - ``catkin init --workspace .``
  - ``catkin config --init``
  - ``mkdir src && catkin build``

... with a default layout in a *different* directory:
  - ``catkin init --workspace /tmp/path/to/my_catkin_ws``

... which explicity extends another workspace:
  - ``catkin config --init --extend /opt/ros/hydro``

Initialize a workspace with a **source space** called ``other_src``:
  - ``catkin config --init --source-space other_src``

... or a workspace with **build**, **devel**, and **install space** ending with the suffix ``_alternate``:
  - ``catkin config --init --space-suffix _alternate``

Configuring Workspaces
^^^^^^^^^^^^^^^^^^^^^^

View the current configuration:
  - ``catkin config``

Setting and un-setting CMake options:
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

Build packages with aditional CMake args:
  - ``catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug``

... and save them to be used for the next build:
  - ``catkin build --save-config --cmake-args -DCMAKE_BUILD_TYPE=Debug``

Cleaning Build Products
^^^^^^^^^^^^^^^^^^^^^^^

Blow away the build, devel, and install spaces (if they exist):
  - ``catkin clean -a``

... or just the **build space**:
  - ``catkin clean --build``

... or just delete the `CMakeCache.txt` files for each package:
  - ``catkin clean --cmake-cache``

... or just delete the build directories for packages which have been disabled or removed:
  - ``catkin clean --orphans``

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

    catkin clean -a
    catkin config --extend /opt/ros/hydro

Change from explicit to implicit chaining:
  .. code-block:: bash

    catkin clean -a
    catkin config --no-extend
    
Migrating from ``catkin_make`` or ``catkin_make_isolated``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Space Options
-------------

Previously, you could specify the layout of a catkin workspace with arguments
such as ``--source``, ``--build``, and ``--devel``. These have been removed
from the ``catkin build`` verb since they are considered properties of the
workspace configuration. As such, if you wish to use a different **source
space**, or change defaults for other spaces, you should use the appropriate
``catkin config`` calls.

  .. code-block:: bash
    
    catkin_make_isolated --cmake-args -DCMAKE_BUILD_TYPE=Debug --build build_dbg --devel devel_dbg

  .. code-block:: bash
    
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug --build build_dbg --devel devel_dbg
    catkin build

Migrating from ``catkin_make``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Workspace Structure
-------------------

- There is no longer a "top-level" ``CMakeLists.txt`` file. The **source
  space** simply contains a collection of packages.

Design Considerations
---------------------

- You can no longer access variables defined in other Catkin projects.
- You no longer need to define target dependencies on messages built in other
  packages. All targets in a dependency are guaranteed to have been built before
  the current package.

