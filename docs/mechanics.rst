Workspace Mechanics
===================

This chapter defines the organization, composition, and use of Catkin workspaces.
Catkin workspaces enable rapid simultaneous building and executing of numerous interdependent projects.
These projects do not need to share the same build tool, but they do need to be able to either build or install to a FHS tree.

Unlike integrated development environments (IDEs) which normally only manage single projects, the purpose of Catkin is to enable the simultaneous compilation of numerous independently-authored projects.

Workspace Configuration
^^^^^^^^^^^^^^^^^^^^^^^

Most ``catkin`` commands which modify a workspace's configuration will
display the standard configuration summary, as shown below:

.. code-block:: bash

    $ cd /tmp/path/to/my_catkin_ws
    $ catkin config
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    --------------------------------------------------------------
    Source Space:       [exists] /tmp/path/to/my_catkin_ws/src
    Log Space:         [missing] /tmp/path/to/my_catkin_ws/logs
    Build Space:       [missing] /tmp/path/to/my_catkin_ws/build
    Devel Space:       [missing] /tmp/path/to/my_catkin_ws/devel
    Install Space:      [unused] /tmp/path/to/my_catkin_ws/install
    DESTDIR:            [unused] None
    --------------------------------------------------------------
    Devel Space Layout:          linked
    Install Space Layout:        merged
    --------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    Internal Make Job Server:    True
    Cache Job Environments:      False
    --------------------------------------------------------------
    Whitelisted Packages:        None
    Blacklisted Packages:        None
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------

This summary describes the layout of the workspace as well as other important settings which influence build and execution behavior.
Each of these options can be modified either with the ``config`` verb's options described in the full command-line usage or by changing environment variables.
The summary is composed of the following sections:

Overview Section
----------------

- **Profile** -- The name of this configuration.
- **Extending** -- Describes if your current configuration will extend another Catkin workspace, and through which mechanism it determined the location of the extended workspace:

  - *No Chaining*
  - *Implicit Chaining* -- Derived from the ``CMAKE_PREFIX_PATH`` environment variable.
  - *Cached Implicit Chaining* -- Derived from the ``CMAKE_PREFIX_PATH`` CMake cache variable.
  - *Explicit Chaining* -- Specified by ``catkin config --extend``

- **Workspace** -- The path to the workspace.
- **Source Space** -- The subdirectory containing the source packages.
- **Build Space** -- The subdirectory containing the intermediate build products for each package.
- **Devel Space** -- The subdirectory containing the final build products which can be used to run code, but relies on the presence of the source space.
- **Install Space** -- The subdirectory containing the final build products which can be used to run code, but is entirely self-contained.
- **DESTDIR** -- An optional prefix to the **install space** as defined by `GNU Standards <https://www.gnu.org/prep/standards/html_node/DESTDIR.html>`_

Build Product Layout Section
----------------------------

- **Devel Space Layout** -- The organization of the **devel space**.

  - *Linked* -- Write products from each package into independent isolated FHS trees, and symbolically link them into a merged FHS tree.
    For more details, see :doc:`Linked Devel Space <advanced/linked_develspace>`.
  - *Merged* -- Write products from all packages to a single FHS tree. This is most similar to the behavior of ``catkin_make``.
  - *Isolated* -- Write products from each package into independent isolated FHS trees. this is most similar to the behavior of ``catkin_make_isolated``.

- **Install Packages** -- Enable creating and installation into the **install space**.
- **Isolate Installs** -- Installs products into individual FHS subdirectories in the **install space**.

Build Tool Arguments Section
----------------------------

- **Additional CMake Args** -- Arguments to be passed to CMake during the *configuration* step for all packages to be built.
- **Additional Make Args** -- Arguments to be passed to Make during the *build* step for all packages to be built.
- **Additional catkin Make Args** -- Similar to **Additional Make Args** but only applies to Catkin packages.
- **Internal Make Job Server** -- Whether or not the internal job server should be used to coordinate parallel build jobs.
- **Cache Job Environments** -- Whether or not environment variables should be cached between build jobs.

Package Filter Section
----------------------

- **Package Whitelist** -- Packages that will be built with a bare call to ``catkin build``.
- **Package Blacklist** -- Packages that will *not* be built unless explicitly named.

Notes Section
-------------

The summary will sometimes contain notes about the workspace or the action that you're performing, or simply tell you that the workspace configuration appears valid.

Warnings Section
----------------

If something is wrong with your configuration such as a missing source space, an additional section will appear at the bottom of the summary with details on what is wrong and how you can fix it.

Workspace Anatomy
^^^^^^^^^^^^^^^^^

A standard catkin workspace, as defined by `REP-0128 <http://www.ros.org/reps/rep-0128.html>`_, is a directory with a prescribed set of "spaces", each of which is contained within a directory under the workspace root.
The spaces that comprise the workspace are described in the following sections.
In addition to the directories specified by `REP-0128 <http://www.ros.org/reps/rep-0128.html>`_, ``catkin_tools`` also adds a visible ``logs`` directory and a hidden ``.catkin_tools`` directory.
The ``.catkin_tools`` directory stores persistent build configuration and profiles.

===============  ===============  ======================================================
 Space            Default Path     Contents
===============  ===============  ======================================================
 Source Space     ``./src``        Source code for all the packages.
 Log Space        ``./logs``       Logs from building and cleaning packages.
 Build Space      ``./build``      Intermediate build products for each package.
 Devel Space      ``./devel``      FHS tree or trees containing all final build products.
 Install Space    ``./install``    FHS tree or trees containing products of ``install`` targets.
===============  ===============  ======================================================

source space
------------

The **source space** contains the source code for all of the packages to be built in the workspace, as such, it is the only directory required to build a workspace.
The **source space** is also the only directory in the catkin workspace which is not modified by any ``catkin`` command verb.
No build products are written to the source space, they are all built "out-of-source" in the **build space**, described in the next section.
You can consider the **source space** to be read-only.

log space
---------

The ``catkin`` command generates a log space, called ``logs`` by default, which contains build logs for each package.
Logs for each package are written in subdirectories with the same name as the package.

The latest log for each verb and stage in a given package's log directory is also written with the format:

.. code-block:: bash

   {VERB}.{STAGE}.log

Each previous logfile has the following format, where ``{INDEX}`` begins at ``000`` and increases with each execution of that verb and stage:

.. code-block:: bash

   {VERB}.{STAGE}.{INDEX}.log


build space
-----------

Intermediate build products are written in the **build space**.
The **build space** contains an isolated build directory for each package, as well as the log files which capture the output from each build stage.
It is from these directories where commands like ``cmake`` and ``make`` are run.

devel space
-----------

Build products like executables, libraries, pkg-config files, and CMake config files, are generated in the **devel space**.
The **devel space** is organized as an `FHS <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_ tree.

Some build tools simply treat the **devel space** as an install prefix, but other buildtools like ``catkin``, itself, can build targets directly into the **devel space** in order to skip the additional install step.
For such packages, executing programs from the **devel space** sometimes requires that the source space is still available.

At the root of the **devel space** is a set of environment setup files which can be "sourced" in order to properly execute the space's products.

install space
-------------

Finally, if the workspace is configured to install packages, the each will be installed into the **install space**.
The **install space** has an FHS layout like the **devel space**, except it is entirely self-contained.

Additional Files Generated by ``catkin_tools``
----------------------------------------------

Configuration Directory
~~~~~~~~~~~~~~~~~~~~~~~

In addition to the standard workspace structure, ``catkin_tools`` also adds a marker directory called ``.catkin_tools`` at the root of the workspace.
This directory both acts as a marker for the root of the workspace and contains persistent configuration information.

This directory contains subdirectories representing different configuration profiles, and inside of each profile directory are YAML files which contain verb-specific metadata.
It additionally contains a file which lists the name of the active configuration profile if it is different from ``default``.

Environment Setup Files
~~~~~~~~~~~~~~~~~~~~~~~

The FHS trees of the **devel space** and **install space** also contain several environment "setup" scripts.
These setup scripts are intended to make it easier to use the resulting FHS tree for building other source code or for running programs built by the packages in the workspace.

The setup script can be used like this in ``bash``:

.. code-block:: bash

    $ source /path/to/workspace/devel/setup.bash

Or like this in ``zsh``:

.. code-block:: bash

    % source /path/to/workspace/devel/setup.zsh

Sourcing these setup scripts adds this workspace and any "underlaid" workspaces to your environment, prefixing several environment variables with the appropriate local workspace folders.

============================= ==================================================
 Environment Variable         | Description
============================= ==================================================
 CMAKE_PREFIX_PATH_           | Used by CMake to find development packages, \
                              | and used by Catkin for workspace chaining.
----------------------------- --------------------------------------------------
 CPATH_ [4]_                  | Used by GCC to search for development headers.
----------------------------- --------------------------------------------------
 LD_LIBRARY_PATH_ [1]_        | Search path for dynamically loadable libraries.
----------------------------- --------------------------------------------------
 DYLD_LIBRARY_PATH_ [2]_      | Search path for dynamically loadable libraries.
----------------------------- --------------------------------------------------
 PATH_                        | Search path for executables.
----------------------------- --------------------------------------------------
 PKG_CONFIG_PATH_             | Search path for ``pkg-config`` files.
----------------------------- --------------------------------------------------
 PYTHONPATH_                  | Search path for Python modules.
============================= ==================================================

.. _CMAKE_PREFIX_PATH: https://cmake.org/cmake/help/v3.0/variable/CMAKE_PREFIX_PATH.html
.. _CPATH: https://gcc.gnu.org/onlinedocs/cpp/Environment-Variables.html
.. _LD_LIBRARY_PATH: http://tldp.org/HOWTO/Program-Library-HOWTO/shared-libraries.html#AEN80
.. _DYLD_LIBRARY_PATH: https://developer.apple.com/library/mac/documentation/Darwin/Reference/ManPages/man1/dyld.1.html
.. _PATH: https://en.wikipedia.org/wiki/PATH_(variable)
.. _PKG_CONFIG_PATH: http://linux.die.net/man/1/pkg-config
.. _PYTHONPATH: https://docs.python.org/2/using/cmdline.html#envvar-PYTHONPATH
.. _changelog: https://github.com/ros/catkin/blob/kinetic-devel/CHANGELOG.rst#070-2016-03-04

.. [1] GNU/Linux Only
.. [2] Mac OS X Only
.. [4] Only in versions of ``catkin`` <= ``0.7.0`` (ROS Kinetic), see the changelog_


The setup scripts will also execute any Catkin "env-hooks" exported by packages
in the workspace. For example, this is how ``roslib`` sets the
``ROS_PACKAGE_PATH`` environment variable.

.. note::

    Like the **devel space**, the **install space** includes ``setup.*`` and
    related files at the top of the file hierarchy.
    This is not suitable for some packaging systems, so this can be disabled by
    passing the ``-DCATKIN_BUILD_BINARY_PACKAGE="1"`` option to ``cmake`` using
    the ``--cmake-args`` option for this verb.
    Though this will suppress the installation of the setup files, you will
    loose the functionality provided by them, namely extending the environment
    and executing environment hooks.

Source Packages and Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A package is any folder which contains a ``package.xml`` as defined by the ROS
community in ROS Enhancement Proposals
`REP-0127 <https://github.com/ros-infrastructure/rep/blob/master/rep-0127.rst>`_
and
`REP-0140 <https://github.com/ros-infrastructure/rep/blob/master/rep-0140.rst>`_.

The ``catkin build`` command builds packages in the topological order determined by the dependencies listed in the package's ``package.xml`` file.
For more information on which dependencies contribute to the build order, see the :doc:`build verb documentation<verbs/catkin_build>`.

Additionally, the ``build_type`` tag is used to determine which build stages to use on the package.
Supported build types are listed in :doc:`Build Types <build_types>`.
Packages without a ``build_type`` tag are assumed to be catkin packages.

For example, plain CMake packages can be built by adding a ``package.xml`` file to the root of their source tree with the ``build_type`` flag set to ``cmake`` and appropriate ``build_depend`` and ``run_depend`` tags set, as described in `REP-0136 <http://www.ros.org/reps/rep-0136.html>`_.
This can been done to build packages like ``opencv``, ``pcl``, and ``flann``.

Workspace Chaining / Extending
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

An important property listed in the configuration which deserves attention is the summary value of the ``Extending`` property.
This affects which other collections of libraries and packages which will be visible to your workspace.
This is process called "workspace chaining."

Above, it's mentioned that the Catkin setup files export numerous environment variables, including ``CMAKE_PREFIX_PATH``.
Since CMake 2.6.0, the ``CMAKE_PREFIX_PATH`` is used when searching for include files, binaries, or libraries using the ``FIND_PACKAGE()``, ``FIND_PATH()``, ``FIND_PROGRAM()``, or ``FIND_LIBRARY()`` CMake commands.

As such, this is also the primary way that Catkin "chains" workspaces together.
When you build a Catkin workspace for the first time, it will automatically use ``CMAKE_PREFIX_PATH`` to find dependencies.
After that compilation, the value will be cached internally by each project as well as the Catkin setup files and they will ignore any changes to your ``CMAKE_PREFIX_PATH`` environment variable until they are cleaned.

.. note::

  Workspace **chaining** is the act of putting the products of one workspace   ``A`` in the search scope of another workspace ``B``.
  When describing the   relationship between two such chained workspaces, ``A`` and ``B``, it is said   that workspace ``B`` **extends** workspace ``A`` and workspace ``A`` is   **extended by** workspace ``B``.
  This concept is also sometimes referred to   as "overlaying" or "inheriting" a workspace.

Similarly, when you ``source`` a Catkin workspace's setup file from a workspace's **devel space** or **install space**, it prepends the path containing that setup file to the ``CMAKE_PREFIX_PATH`` environment variable.
The next time you initialize a workspace, it will extend the workspace that you previously sourced.

This makes it easy and automatic to chain workspaces.
Previous tools like ``catkin_make`` and ``catkin_make_isolated`` had no easy mechanism for either making it obvious which workspace was being extended, nor did they provide features to explicitly extend a given workspace.
This means that for users were unaware of Catkin's use of ``CMAKE_PREFIX_PATH``.

Since it's not expected that 100% of users will read this section of the documentation, the ``catkin`` program adds both configuration consistency checking for the value of ``CMAKE_PREFIX_PATH`` and  makes it obvious on each invocation which workspace is being extended.
Furthermore, the ``catkin`` command adds an explicit extension interface to override the value of ``$CMAKE_PREFIX_PATH`` with the ``catkin config --extend`` command.

 .. note::

  While workspaces can be chained together to add search paths, invoking a   build in one workspace will not cause products in any other workspace to be   built.

The information about which workspace to extend can come from a few different sources, and can be classified in one of three ways:

No Chaining
-----------

This is what is shown in the above example configuration and it implies that there are no other Catkin workspaces which this workspace extends.
The user has neither explicitly specified a workspace to extend, and the ``CMAKE_PREFIX_PATH`` environment variable is empty:

.. code-block:: bash

      Extending:                   None

Implicit Chaining via ``CMAKE_PREFIX_PATH`` Environment or Cache Variable
-------------------------------------------------------------------------

In this case, the ``catkin`` command is *implicitly* assuming that you want to build this workspace against resources which have been built into the directories listed in your ``CMAKE_PREFIX_PATH`` environment variable.
As such, you can control this value simply by changing this environment variable.

For example, ROS users who load their system's installed ROS environment by calling something similar to ``source /opt/ros/indigo/setup.bash`` will normally see an ``Extending`` value such as:

.. code-block:: bash

      Extending:             [env] /opt/ros/indigo

If you don't want to extend the given workspace, unsetting the ``CMAKE_PREFIX_PATH`` environment variable will change it back to none.
Once you have built your workspace once, this ``CMAKE_PREFIX_PATH`` will be cached by the underlying CMake buildsystem.
As such, the ``Extending`` status will subsequently describe this as the "cached" extension path:

.. code-block:: bash

      Extending:          [cached] /opt/ros/indigo

Once the extension mode is cached like this, you must use ``catkin clean`` to before changing it to something else.

Explicit Chaining via ``catkin config --extend``
------------------------------------------------

This behaves like the above implicit chaining except it means that this workspace is *explicitly* extending another workspace and the workspaces which the other workspace extends, recursively.
This can be set with the ``catkin config --extend`` command.
It will override the value of ``CMAKE_PREFIX_PATH`` and persist between builds.

.. code-block:: bash

      Extending:        [explicit] /tmp/path/to/other_ws
