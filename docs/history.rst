
A Brief History of Catkin
=========================

Legacy Catkin Workflow
^^^^^^^^^^^^^^^^^^^^^^

The core Catkin meta-buildsystem was originally designed in order to efficiently build numerous inter-dependent, but separately developed, CMake projects.
This was developed by the Robot Operating System (ROS) community, originally as a successor to the standard meta-buildtool ``rosbuild``.
The ROS community's distributed development model with many modular projects and the need for building distributable binary packages motivated the design of a system which efficiently merged numerous disparate projects so that they utilize a single target dependency tree and build space.

To facilitate this "merged" build process, a workspace's **source space** would contain boiler-plate "top-level" ``CMakeLists.txt`` which automatically added all of the Catkin CMake projects below it to the single large CMake project.

Then the user would build this collection of projects like a single unified CMake project with a workflow similar to the standard CMake out-of-source build workflow.
They would all be configured with one invocation of ``cmake`` and subsequently targets would be built with one or more invocations of ``make``:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src
    $ make

In order to help automate the merged build process, Catkin was distributed with a command-line tool called ``catkin_make``.
This command automated the above CMake work flow while setting some variables according to standard conventions.
These defaults would result in the execution of the following commands:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src -DCATKIN_DEVEL_SPACE=../devel -DCMAKE_INSTALL_PREFIX=../install
    $ make -j<number of cores> -l<number of cores> [optional target, e.g. install]

An advantage of this approach is that the total configuration would be smaller than configuring each package individually and that the Make targets can be parallelized even among dependent packages.

In practice, however, it also means that in large workspaces, modification of the CMakeLists.txt of one package would necessitate the reconfiguration of all packages in the entire workspace.

A critical flaw of this approach, however, is that there is no fault isolation.
An error in a leaf package (package with no dependencies) will prevent all packages from configuring.
Packages might have colliding target names.
The merged build process can even cause CMake errors to go undetected if one package defines variables needed by another one, and can depend on the order in which independent packages are built.
Since packages are merged into a single CMake invocation, this approach also requires developers to specify explicit dependencies on some targets inside of their dependencies.

Another disadvantage of the merged build process is that it can only work on a homogeneous workspace consisting only of Catkin CMake packages.
Other types of packages like plain CMake packages and autotools packages cannot be integrated into a single configuration and a single build step.

Isolated Catkin Workflow
^^^^^^^^^^^^^^^^^^^^^^^^

The numerous drawbacks of the merged build process and the ``catkin_make`` tool motivated the development of the ``catkin_make_isolated`` tool.
In contrast to ``catkin_make``, the ``catkin_make_isolated`` command uses an isolated build process, wherein each package is independently configured, built, and loaded into the environment.

This way, each package is built in isolation and the next packages are built on the atomic result of the current one.
This resolves the issues with target collisions, target dependency management, and other undesirable cross-talk between projects.
This also allows for the homogeneous automation of other buildtools like the plain CMake or autotools.

The isolated workflow also enabled the following features:

- Allowing building of *part* of a workspace
- Building Catkin and non-Catkin projects into a single **devel space**
- Building packages without re-configuring or re-building their dependencies
- Removing the requirement that all packages in the workspace are free
  of CMake errors before any packages can be built

There are, however, still some problems with ``catkin_make_isolated``.
First, it is dramatically slower than ``catkin_make`` since it cannot parallelize the building of targets or even packages which do not depend on each other.
It also lacks robustness to changes in the list of packages in the workspace.
Since it is a "released" tool, it also has strict API stability requirements.

Parallel Isolated Catkin Workflow and ``catkin build``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The limitations of ``catkin_make_isolated`` and the need for additional high-level build tools lead to the development of a parallel version of catkin make isolated, or ``pcmi``, as part of `Project Tango <http://osrfoundation.org/blog/project-tango-announced.html>`_.
``pcmi`` later became the ``build`` verb of the ``catkin`` command included in this project.

As such, the principle behavior of the ``build`` verb is to build each package in isolation and in topological order while parallelizing the building of packages which do not depend on each other.

Other functional improvements over ``catkin_make`` and ``catkin_make_isolated`` include the following: 

- The use of sub-command "verbs" for better organization of build options and
  build-related functions
- Robustly adapting a build when packages are added to or removed from the
  **source space**
- Context-aware building of a given package based on the working directory
- The ability to completely clean a single package's products from a workspace
- Utilization of persistent build metadata which catches common errors
- Support for different build "profiles" in a single workspace
- Explicit control of workspace chaining
- Additional error-checking for common environment configuration errors
- Numerous other command-line user-interface improvements

