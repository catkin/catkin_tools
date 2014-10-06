Workspace Mechanics
===================

This chapter defines how Catkin workspaces are organized and used, as well as
some standardized nomenclature for describing elements of a Catkin workspace.
Unlike integrated development environments (IDEs) which normally only manage
single projects, the purpose of Catkin is to enable the simultaneous
compilation of numerous independently-authored projects. As such, these
projects need to be organized in a "workspace" which contains both the source
and build products for a collection of "packages."


Anatomy of a Catkin Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A standard catkin workspace, as defined by `REP-0128
<http://www.ros.org/reps/rep-0128.html>`_, is a folder with a prescribed set
"spaces", each of which is normally a folder within the workspace:

- **source space** -- default: ``./src``
- **build space** -- default: ``./build``
- **devel space** -- default: ``./devel``
- **install space** -- default: ``./install``

Though there are standard conventions for the layout and names of the
workspace's various spaces, they can be renamed for a given build
using the ``catkin config`` verb.

source space
------------

The **source space** is where the code for your packages resides and normally
is in the folder ``/path/to/workspace/src``.  The build command considers
**source space** to be read-only, in that during a build no files or folders
should be created or modified in that folder.  Therefore catkin workspaces
are said to be built "out of source", which simply means that the folder in
which you build your code is not under or part of the folder with contains
the source code.

build space
-----------

Temporary build files are put into the **build space**, which by default is in
the ``/path/to/workspace/build`` folder.  The **build space** is the working
directory in which commands like ``cmake`` and ``make`` are run.

devel space
-----------

Generated files, like executables, libraries, pkg-config files, CMake config
files, or message code, are placed in the **devel space**.
By convention the **devel space** is located as a peer to the **source
space** and **build space** in the ``/path/to/workspace/devel`` folder.
The layout of the **devel space** is intended to mimic the root of an `FHS
filesystem <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_,
with folders like ``include``, ``lib``, ``bin``, and ``share``. Running code is
possible from **devel space** because references to the **source space** are
created.

install space
-------------

Finally, if the packages in the workspace are setup for installing, the
``--install`` option can be invoked to install the packages to the
``CMAKE_INSTALL_PREFIX``, which in `REP-0128
<http://www.ros.org/reps/rep-0128.html>`_ terms is the **install space**.
The **install space**, like the **devel space**, has an FHS layout along with
some generated setup files.
The **install space** is set to ``/path/to/workspace/install`` by changing
the ``CMAKE_INSTALL_PREFIX`` by default.
This is done to prevent users from accidentally trying to install to the
normal ``CMAKE_INSTALL_PREFIX`` path, ``/usr/local``.
Unlike the **devel space**, the **install space** is completely standalone
and does not require the **source space** or **build space** to function, and
is suitable for packaging.

Additional Workspace Directories with ``catkin_tools``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Hidden Marker / Config Directory
--------------------------------

In addition to the standard workspace structure, ``catkin_tools`` also adds a
marker directory called ``.catkin_tools`` at the root of the workspace. This
directory both acts as a marker for the root of the workspace and contains
persistent configuration information.

This directory contains subdirectories representing different configuration
profiles, and inside of each profile directory are YAML files which contain
verb-specific metadata. It additionally contains a file which lists the name of
the active configuration profile if it is different than ``default``.

Build Log Directory
-------------------

Another addition is the ``build_logs`` directory which is generated in the
**build space** and contains individual build logs for each package.


Environment Setup Files
^^^^^^^^^^^^^^^^^^^^^^^

In addition to the FHS folders, some setup scripts are generated in the **devel
space**, e.g. ``setup.bash`` or ``setup.zsh``.
These setup scripts are intended to make it easier to use the resulting **devel
space** for building on top of the packages that were just built or for running
programs built by those packages.
The setup script can be used like this in ``bash``:

.. code-block:: bash

    $ source /path/to/workspace/devel/setup.bash

Or like this in ``zsh``:

.. code-block:: bash

    % source /path/to/workspace/devel/setup.zsh

Sourcing these setup scripts adds this workspace and any "underlaid"
workspaces to your environment, prefixing the ``CMAKE_PREFIX_PATH``,
``PKG_CONFIG_PATH``, ``PATH``, ``LD_LIBRARY_PATH``, ``CPATH``, and
``PYTHONPATH`` with local workspace folders.

The setup scripts will also execute any shell hooks exported by packages in the
workspace, which is how ``roslib``, for example, sets the ``ROS_PACKAGE_PATH``
environment variable.

.. note::

    Like the **devel space**, the **install space** includes ``setup.*`` and
    related files at the top of the file hierarchy.
    This is not suitable for some packaging systems, so this can be disabled by
    passing the ``-DCATKIN_BUILD_BINARY_PACKAGE="1"`` option to ``cmake`` using
    the ``--cmake-args`` option for this verb.
    Though this will suppress the installation of the setup files, you will
    loose the functionality provided by them, namely extending the environment
    and executing environment hooks.

Workspace Packages and Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A workspace's packages consist of any packages found in the **source space**.
A package is any folder which contains a ``package.xml`` as defined in
`REP-0127 <http://www.ros.org/reps/rep-0127.html>`_.

The ``catkin build`` command determines the order in which packages are built
based on the ``depend``, ``build_depend``, ``run_depend``, and ``build_type``
tags in a package's ``package.xml`` file.

- The ``*_depend`` tags are used to determine the topological build order of
  the packages.
- The ``build_type`` tag is used to determine which build work flow to use on
  the package.

Packages without an explicitly defined ``build_type`` tag are assumed to be
catkin packages, but plain CMake packages can be built by adding a
``package.xml`` file to the root of their source tree with the ``build_type``
flag set to ``cmake`` and appropriate ``build_depend`` and ``run_depend`` tags
set, as described in `REP-0136 <http://www.ros.org/reps/rep-0136.html>`_.
This has been done in the past for building packages like ``opencv``, ``pcl``,
and ``flann``.

Understanding the Build Process
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Legacy Catkin Workflow
----------------------

The core Catkin meta-buildsystem was originally designed in order to
efficiently build numerous inter-dependent, but separately developed, CMake
projects. This was developed by the Robot Operating System (ROS)
community, originally as a successor to the standard meta-buildtool
``rosbuild``. The ROS community's distributed development model with many
modular projects and the need for building distributable binary packages
motivated the design of a system which efficiently merged numerous disparate
projects so that they utilize a single target dependency tree and build space.

To facilitate this "merged" build process, a workspace's **source space**
would contain boiler-plate "top-level" ``CMakeLists.txt`` which automatically
added all of the Catkin CMake projects below it to the single large CMake
project.

Then the user would build this collection of projects like a single unified
CMake project with a workflow similar to the standard CMake out-of-source build
workflow. They would all be configured with one invocation of ``cmake`` and
subsequently targets would be built with one or more invocations of ``make``:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src
    $ make

In order to help automate the merged build process, Catkin was distributed
with a command-line tool called ``catkin_make``.
This command automated the above CMake work flow while setting some
variables according to standard conventions.
These defaults would result in the execution of the following commands:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src -DCATKIN_DEVEL_SPACE=../devel -DCMAKE_INSTALL_PREFIX=../install
    $ make -j<number of cores> -l<number of cores> [optional target, e.g. install]

An advantage of this approach is that the total configuration would be smaller
than configuring each package individually and that the Make targets can be
parallelized even amongst dependent packages.

In practice, however, it also means that in large workspaces, modification of the
CMakeLists.txt of one package would necessitate the reconfiguration of all packages
in the entire workspace.

A critical flaw of this approach, however, is that there is no fault isolation.
An error in a leaf package (package with no dependencies) will prevent all
packages from configuring. Packages might have colliding target names. The
merged build process can even cause CMake errors to go undetected if one package
defines variables needed by another one, and can depend on the order in which
independent packages are built. Since packages are merged into a single CMake
invocation, this approach also requires developers to specify explicit
dependencies on some targets inside of their dependencies.

Another disadvantage of the merged build process is that it can only work on a
homogeneous workspace consisting only of Catkin CMake packages.
Other types of packages like plain CMake packages and autotools packages cannot
be integrated into a single configuration and a single build step.

Isolated Catkin Workflow
------------------------

The numerous drawbacks of the merged build process and the ``catkin_make`` tool
motivated the development of the ``catkin_make_isolated`` tool.
In contrast to ``catkin_make``, the ``catkin_make_isolated`` command uses an
isolated build process, wherein each package is independently configured,
built, and loaded into the environment.

This way, each package is built in isolation and the next packages are built on
the atomic result of the current one. This resolves the issues with target
collisions, target dependency management, and other undesirable cross-talk 
between projects.
This also allows for the homogeneous automation of other buildtools like the
plain CMake or autotools.

The isolated workflow also enabled the following features:

- Allowing building of *part* of a workspace
- Building Catkin and non-Catkin projects into a single **devel space**
- Building packages without re-configuring or re-building their dependencies
- Removing the requirement that all packages in the workspace are free
  of CMake errors before any packages can be built 

There are, however, still some problems with ``catkin_make_isolated``. First,
it is dramatically slower than ``catkin_make`` since it cannot parallelize the
building of targets or even packages which do not depend on each other.
It also lacks robustness to changes in the list of packages in the
workspace. Since it is a "released" tool, it also has strict API stability
requirements.

Parallel Isolated Catkin Workflow and ``catkin build``
------------------------------------------------------

The limitations of ``catkin_make_isolated`` and the need for additional
high-level build tools lead to the development of a parallel version of 
catkin make isolated, or ``pcmi``, as part of `Project
Tango <http://osrfoundation.org/blog/project-tango-announced.html>`_.
``pcmi`` later became the ``build`` verb of the ``catkin`` command included
in this project.

As such, the principle behavior of the ``build`` verb is to build each
package in isolation and in topological order while parallelizing the
building of packages which do not depend on each other.

Other functional improvements over ``catkin_make`` and ``catkin_make_isolated``
include the following:

- The use of sub-command "verbs" for better organization of build options and 
  build-related functions
- Robustly adapting a build when packages are added to or removed from the
  **source space**
- Context-aware building of a given package based on the working directory
- Utilization of persistent build metadata which catches common errors
- Support for different build "profiles" in a single workspace
- Explicit control of workspace chaining
- Additional error-checking for common environment configuration errors
- Numerous other command-line user-interface improvements

Workspace Chaining and the Importance of CMAKE_PREFIX_PATH
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Above, it's mentioned that the Catkin setup files export numerous environment
variables, including ``CMAKE_PREFIX_PATH``. Since CMake 2.6.0, the
``CMAKE_PREFIX_PATH`` is used when searching for include files, binaries, or
libraries using the ``FIND_PACKAGE()``, ``FIND_PATH()``, ``FIND_PROGRAM()``, or
``FIND_LIBRARY()`` CMake commands.

As such, this is also the primary way that Catkin "chains" workspaces together.
When you build a Catkin workspace for the first time, it will automatically use
``CMAKE_PREFIX_PATH`` to find dependencies. After that compilation, the value
will be cached internally by each project as well as the Catkin setup files and
they will ignore any changes to your ``CMAKE_PREFIX_PATH`` environment variable
until they are cleaned.

.. note::

  Workspace **chaining** is the act of putting the products of one workspace
  ``A`` in the search scope of another workspace ``B``. When describing the
  relationship between two such chained workspaces, ``A`` and ``B``, it is said
  that workspace ``B`` **extends** workspace ``A`` and workspace ``A`` is
  **extended by** workspace ``B``. This concept is also sometimes referred to
  as "overlaying" or "inheriting" a workspace.

Similarly, when you ``source`` a Catkin workspace's setup file from a
workspace's **devel space** or **install space**, it prepends the path
containing that setup file to the ``CMAKE_PREFIX_PATH`` environment variable.
The next time you initialize a workspace, it will extend the workspcae that you
previously sourced.

On one hand, this makes it easy and automatic to chain workspaces. At the same
time, however, previous tools like ``catkin_make`` and ``catkin_make_isolated``
had no easy mechanism for either making it obvious which workspace was being
extended, nor did they provide features to explicitly extend a given workspace.
This means that for users unaware of Catkin's use of ``CMAKE_PREFIX_PATH``

Since it's not expected that 100% of users will read this section of the
documentation, the ``catkin`` program adds both configuration consistency
checking for the value of ``CMAKE_PREFIX_PATH`` and  makes it obvious on each
invocation which workspace is being extended.  Furthermore, the ``catkin``
command adds an explicit extension interface to override the value of
``$CMAKE_PREFIX_PATH`` with the ``catkin config --extend`` command.

.. note::

  While workspaces can be chained together to add search paths, invoking a
  build in one workspace will not cause products in any other workspace to be
  built.


