``catkin config`` -- Configure a Workspace
==========================================

The ``config`` verb can be used to both view and mapiulate a workspace's
configuration options. These options include all of the elements listed in thr
configuration summary.

By default, the ``config`` verb gets and sets options for a workspace's
*active* profile. If no profiles have been specified for a wotkspace, this is a
default profile named ``default``.

.. note::

  Calling ``catkin config`` on an uninitialied workspace will not automatically
  initialize it unless it is used with the ``--init`` option.

Viewing the Configuration Summary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once a workspace has been initialized, the configuration summary can be
displayed by calling ``catkin config`` without arguments from anywhere under
the root of the workspace. Doing so will not modify your workspace. The
``catkin`` command is context-sensitive, so it will determine which workspace
contains the current working directory.

Installing Packages
^^^^^^^^^^^^^^^^^^^

Without any additional arguments, packages are not "installed" using the
standard CMake ``install()`` targets.  Addition of the ``--install`` option
will configure a workspace so that it creates an **install space** and write
the products of all install targets to that FHS tree. The contents of the
**install space**, which, by default, is located in a directory named
``install`` will look like the following:

.. code-block:: none

    $ ls ./install
    _setup_util.py bin            env.sh         etc            include
    lib            setup.bash     setup.sh       setup.zsh      share

Explicitly Specifying Workspace Chaining
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Normally, a catkin workspace automatically "extends" the other workspaces that
have previously been sourced in your environment. Each time you source a catkin
setup file from a result-space (devel-space or install-space), it sets the
``$CMAKE_PREFIX_PATH`` in your environment, and this is used to build the next
workspace. This is also sometimes referred to as "workspace chaining" and
sometimes the extended workspace is referred to as a "parent" workspace.

With ``catkin config``, you can explicitly set the workspace you want to extend,
using the ``--extend`` argument. This is equivalent to sourcing a setup file,
building, and then reverting to the environment before sourcing the setup file.

Note that in case the desired parent workspace is different from one already
being used, using the ``--extend`` argument also necessitates cleaning the
setup files from your workspace with ``catkin clean``.

For example, regardless of your current environment variable settings (like
``$CMAKE_PREFIX_PATH``), this will build your workspace against the
``/opt/ros/hydro`` install space.

First start with an empty ``CMAKE_PREFIX_PATH`` and initialize, build, and
source a workspace:

.. code-block:: bash

    $ echo $CMAKE_PREFIX_PATH

    $ mkdir -p /tmp/path/to/my_catkin_ws/src
    $ cd /tmp/path/to/my_catkin_ws
    $ catkin init
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    ...
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------

    $ cd /tmp/path/to/my_catkin_ws
    $ catkin create pkg aaa
    $ catkin create pkg bbb
    $ catkin create pkg ccc
    $ catkin build
    ...

    $ source devel/setup.bash
    $ echo $CMAKE_PREFIX_PATH
    /tmp/path/to/my_catkin_ws/devel

    $ catkin config
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    ...
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------

At this point you have a workspace which doesn't extend anything. If you
realize this after the fact, you can explicitly tell it to extend another
workspace. Suppose you wanted to extend a standard ROS system install like
``/opt/ros/hydro``. This can be done with the ``--extend`` option:

.. code-block:: bash


    $ catkin config --extend /opt/ros/hydro
    --------------------------------------------------------------
    Profile:                     default
    Extending:        [explicit] /opt/ros/hydro
    Workspace:                   /tmp/path/to/my_catkin_ws
    Source Space:       [exists] /tmp/path/to/my_catkin_ws/src
    Build Space:       [missing] /tmp/path/to/my_catkin_ws/build
    Devel Space:       [missing] /tmp/path/to/my_catkin_ws/devel
    Install Space:     [missing] /tmp/path/to/my_catkin_ws/install
    DESTDIR:                     None
    --------------------------------------------------------------
    Isolate Develspaces:         False
    Install Packages:            False
    Isolate Installs:            False
    --------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------

    $ catkin clean --setup-files
    $ catkin build
    ...

    $ source devel/setup.bash
    $ echo $CMAKE_PREFIX_PATH
    /tmp/path/to/my_catkin_ws:/opt/ros/hydro

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin config [-h] [--workspace WORKSPACE] [--profile PROFILE] [--init]
                         [--extend EXTEND_PATH | --no-extend] [--mkdirs]
                         [-s SOURCE_SPACE | --default-source-space]
                         [-b BUILD_SPACE | --default-build-space]
                         [-d DEVEL_SPACE | --default-devel-space]
                         [-i INSTALL_SPACE | --default-install-space]
                         [-x SPACE_SUFFIX] [--isolate-devel | --merge-devel]
                         [--install | --no-install]
                         [--isolate-install | --merge-install]
                         [--parallel-jobs PARALLEL_JOBS]
                         [--cmake-args ARG [ARG ...] | --no-cmake-args]
                         [--make-args ARG [ARG ...] | --no-make-args]
                         [--catkin-make-args ARG [ARG ...] |
                         --no-catkin-make-args]

    This verb is used to configure a catkin workspace's configuration and layout.
    Calling `catkin config` with no arguments will display the current config and
    affect no changes if a config already exists for the current workspace and
    profile.

    optional arguments:
      -h, --help            show this help message and exit
      --workspace WORKSPACE, -w WORKSPACE
                            The path to the catkin_tools workspace or a directory
                            contained within it (default: ".")
      --profile PROFILE     The name of a config profile to use (default: active
                            profile)

    Workspace Context:
      Options affecting the context of the workspace.

      --init                Initialize a workspace if it does not yet exist.
      --extend EXTEND_PATH, -e EXTEND_PATH
                            Explicitly extend the result-space of another catkin
                            workspace, overriding the value of $CMAKE_PREFIX_PATH.
      --no-extend           Un-set the explicit extension of another workspace as
                            set by --extend.
      --mkdirs              Create directories required by the configuration (e.g.
                            source space) if they do not already exist.

    Spaces:
      Location of parts of the catkin workspace.

      -s SOURCE_SPACE, --source-space SOURCE_SPACE
                            The path to the source space.
      --default-source-space
                            Use the default path to the source space ("src")
      -b BUILD_SPACE, --build-space BUILD_SPACE
                            The path to the build space.
      --default-build-space
                            Use the default path to the build space ("build")
      -d DEVEL_SPACE, --devel-space DEVEL_SPACE
                            Sets the target devel space
      --default-devel-space
                            Sets the default target devel space ("devel")
      -i INSTALL_SPACE, --install-space INSTALL_SPACE
                            Sets the target install space
      --default-install-space
                            Sets the default target install space ("install")
      -x SPACE_SUFFIX, --space-suffix SPACE_SUFFIX
                            Suffix for build, devel, and install space if they are
                            not otherwise explicitly set.

    Devel Space:
      Options for configuring the structure of the devel space.

      --isolate-devel       Build products from each catkin package into isolated
                            devel spaces.
      --merge-devel         Build products from each catkin package into a single
                            merged devel spaces.

    Install Space:
      Options for configuring the structure of the install space.

      --install             Causes each package to be installed to the install
                            space.
      --no-install          Disables installing each package into the install
                            space.
      --isolate-install     Install each catkin package into a separate install
                            space.
      --merge-install       Install each catkin package into a single merged
                            install space.

    Build Options:
      Options for configuring the way packages are built.

      --parallel-jobs PARALLEL_JOBS, --parallel PARALLEL_JOBS, -p PARALLEL_JOBS
                            Maximum number of packages which could be built in
                            parallel (default is cpu count)
      --cmake-args ARG [ARG ...]
                            Arbitrary arguments which are passes to CMake. It must
                            be passed after other arguments since it collects all
                            following options.
      --no-cmake-args       Pass no additional arguments to CMake.
      --make-args ARG [ARG ...]
                            Arbitrary arguments which are passes to make.It must
                            be passed after other arguments since it collects all
                            following options.
      --no-make-args        Pass no additional arguments to make (does not affect
                            --catkin-make-args).
      --catkin-make-args ARG [ARG ...]
                            Arbitrary arguments which are passes to make but only
                            for catkin packages.It must be passed after other
                            arguments since it collects all following options.
      --no-catkin-make-args
                            Pass no additional arguments to make for catkin
                            packages (does not affect --make-args).

