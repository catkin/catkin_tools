``catkin build`` -- Build Packages
==================================

The ``build`` verb is used to build one or more packages in a catkin workspace.
Like most verbs, ``build`` is context-aware. This means that it can be executed from within any directory contained by an *initialized* workspace.

If a workspace is not yet initialized, ``build`` can initialize it, but only if it is called from the workspace root and the default workspace structure is desired.

Workspaces can also be built from arbitrary working directories if the user specifies the path to the workspace with the ``--workspace`` option.

.. note::

    To set up the workspace and clone the repositories used in the following
    examples, you can use `rosinstall_generator <http://wiki.ros.org/rosinstall_generator>`_ and `wstool <http://wiki.ros.org/wstool>`_. This
    clones all of the ROS packages necessary for building the introductory
    ROS tutorials:

    .. code-block:: bash

        $ mkdir -p /tmp/path/to/my_catkin_ws/src
        $ cd /tmp/path/to/my_catkin_ws
        $ catkin init
        $ cd /tmp/path/to/my_catkin_ws/src
        $ rosinstall_generator --deps ros_tutorials > .rosinstall
        $ wstool update

Basic Usage
^^^^^^^^^^^

Previewing The Build
--------------------

Before actually building anything in the workspace, it is useful to preview
which packages will be built and in what order. This can be done with the
``--dry-run`` option:

.. code-block:: bash

    $ catkin build --dry-run
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
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
    Whitelisted Packages:        None
    Blacklisted Packages:        None
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    - catkin               (catkin)
    - genmsg               (catkin)
    - gencpp               (catkin)
    - genlisp              (catkin)
    - genpy                (catkin)
    - console_bridge       (cmake)
    - cpp_common           (catkin)
    - message_generation   (catkin)
    - message_runtime      (catkin)
    - ros_tutorials        (metapackage)
    - rosbuild             (catkin)
    - rosclean             (catkin)
    - roscpp_traits        (catkin)
    - rosgraph             (catkin)
    - roslang              (catkin)
    - roslaunch            (catkin)
    - rosmaster            (catkin)
    - rospack              (catkin)
    - roslib               (catkin)
    - rosparam             (catkin)
    - rospy                (catkin)
    - rostime              (catkin)
    - roscpp_serialization (catkin)
    - rosunit              (catkin)
    - rosconsole           (catkin)
    - rostest              (catkin)
    - std_msgs             (catkin)
    - geometry_msgs        (catkin)
    - rosgraph_msgs        (catkin)
    - std_srvs             (catkin)
    - xmlrpcpp             (catkin)
    - roscpp               (catkin)
    - roscpp_tutorials     (catkin)
    - rosout               (catkin)
    - rospy_tutorials      (catkin)
    - turtlesim            (catkin)
    Total packages: 36

In addition to the listing the package names and in which order they would be
built, it also displays the buildtool type of each package. Among those listed
above are:

 * **catkin** -- A CMake package which uses Catkin
 * **cmake** -- A "vanilla" CMake package
 * **metapackage** -- A package which contains no build products, but just groups
   other packages together for distribution

Building a Workspace
--------------------

When no packages are given as arguments, ``catkin build`` builds the entire workspace.
It automatically creates directories for a **build space** and a **devel space**:

.. code-block:: bash

    $ catkin build
    Creating buildspace directory, '/tmp/path/to/my_catkin_ws/build'
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
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
    Whitelisted Packages:        None
    Blacklisted Packages:        None
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------
    Found '36' packages in 0.0 seconds.
    Starting ==> catkin
    Starting ==> console_bridge
    Finished <== catkin [ 2.4 seconds ]

    ....

    [build] Finished.
    [build] Runtime: 3 minutes and 54.6 seconds

After the build finishes, the **build space** contains directories containing the intermediate build products for each package, and the **devel space** contains an FHS layout into which all the final build products are written.

.. code-block:: bash

    $ ls ./*
    ./build:
    catkin               genlisp              message_runtime      roscpp
    rosgraph_msgs        rosout               rostest              turtlesim
    build_logs           genmsg               ros_tutorials
    roscpp_serialization roslang              rospack              rostime
    xmlrpcpp             console_bridge       genpy                rosbuild
    roscpp_traits        roslaunch            rosparam             rosunit
    cpp_common           geometry_msgs        rosclean
    roscpp_tutorials     roslib               rospy                std_msgs
    gencpp               message_generation   rosconsole           rosgraph
    rosmaster            rospy_tutorials      std_srvs

    ./devel:
    _setup_util.py bin            env.sh         etc            include
    lib            setup.bash     setup.sh       setup.zsh      share

    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack

.. note::

    The products of ``catkin build`` differ significantly from the behavior of
    ``catkin_make``, for example, which would have all of the build files and
    intermediate build products in a combined **build space** or
    ``catkin_make_isolated`` which would have an isolated FHS directory for
    each package in the **devel space**.

Status Line
-----------

When running ``catkin build`` with default options, it displays a "live" status line similar to the following:

.. code-block:: none

    [build - 20.2] [18/34 complete] [4/4 jobs] [1 queued] [xmlrpcpp:make (66%) - 4.9] ...

The status line stays at the bottom of the screen and displays the continuously-updated progress of the entire build as well as the active build jobs which are still running. It is composed of the following information:

 * ``[build - <T>]`` -- The first block on the left indicates the total elapsed
   build time ``<T>`` in seconds thus far.
 * ``[<M>/<N> complete]``  --  The second block from the left indicates the
   build progress in terms of the number of completed packages, ``<M>`` out of
   the total number of packages to be built ``<N>``.
 * ``[<M>/<N> jobs]`` --  The third block from the left indicates the number of
   active total low-level jobs ``<M>`` out of the total number of low-level
   workers ``<N>``.
 * ``[<N> queued]`` --  The fourth block from the left indicates the number of
   jobs ``<N>`` whose dependencies have already been satisfied and are ready to
   be built.
 * ``[<N> failed]`` --  The fifth block from the left indicates the number of
   jobs ``<N>`` which have failed. This block only appears once one or more
   jobs has failed.
 * ``[<package>:<stage> (<P>%) - <T>]`` -- The remaining blocks show details on
   the active jobs. These include the percent complete, ``<P>``, of the stage,
   if available, as well as the time elapsed building the package, ``<T>``.

When necessary, the status line can be disabled by passing the ``--no-status`` option to ``catkin build``.
This is sometimes required when running ``catkin build`` from within a program that doesn't support the ASCII escape sequences required to reset and re-write the status line.

Console Messages
----------------

Normally, unless an error occurs, the output from each package's build proces
is collected but not printed to the console. All that is printed is a pair of
messages designating the start and end of a package's build. This is formatted
like the following for the ``genmsg`` package:

.. code-block:: none

    ...
    Starting  >>> genmsg
    ...
    Finished  <<< genmsg   [ 0.1 seconds ]
    ...

Error messages are printed whenever a build job writes to ``stderr``.
In such cases, the ``build`` verb will automatically print the captured ``stderr`` buffer under a ``Warnings`` header once the job has completed, similarly to below:

.. code-block:: none

    ...
    ____________________________________________________________________________
    Warnings   << rospack:make /path/to/my_catkin_ws/build/_logs/rospack/build.make.000.log
    In file included from /usr/include/python2.7/Python.h:8:0,
                     from /path/to/my_catkin_ws/src/rospack/src/rospack.cpp:71:
    /usr/include/python2.7/pyconfig.h:1161:0: warning: "_POSIX_C_SOURCE" redefined [enabled by default]
    /usr/include/features.h:164:0: note: this is the location of the previous definition
    /usr/include/python2.7/pyconfig.h:1183:0: warning: "_XOPEN_SOURCE" redefined [enabled by default]
    /usr/include/features.h:166:0: note: this is the location of the previous definition
    ............................................................................
    Finished  <<< rospack                     [ 11.7 seconds ]
    ...

Note that the first line displays the path to the interleaved log file, which persists until the build space is cleaned.
Additionally, if a package fails, the output to ``stderr`` is printed under the ``Errors`` header.

.. code-block:: none

    ____________________________________________________________________________
    Errors     << catkin_pkg_make_err:make /home/jbohren/ws/catkin_tools_test_ws/build/_logs/catkin_pkg_make_err/build.make.062.log
    /home/jbohren/ws/catkin_tools_test_ws/src/catkin_pkg_make_err/main.cpp: In function ‘int main(int, char**)’:
    /home/jbohren/ws/catkin_tools_test_ws/src/catkin_pkg_make_err/main.cpp:3:3: error: expected ‘,’ or ‘;’ before ‘return’
    make[2]: *** [CMakeFiles/main.dir/main.cpp.o] Error 1
    make[1]: *** [CMakeFiles/main.dir/all] Error 2
    make: *** [all] Error 2
    ............................................................................
    Failed     << catkin_pkg_make_err:make     [ Exited with code 2 ]

All of the messages from the underlying jobs can be shown when using the
``-v`` or ``--verbose`` option. This will print the normal messages when a
build job starts and finishes as well as the interleaved output to ``stdout``
and ``stderr`` from each build command in a block.

.. code-block:: none

    Starting  >>> genmsg
    Starting   >> genmsg:mkdir
    Finished   << genmsg:mkdir
    Starting   >> genmsg:envgen
    Finished   << genmsg:envgen
    Starting   >> genmsg:cmake
    Subprocess  > genmsg:cmake `cd /path/to/my_catkin_ws/build/genmsg && /path/to/my_catkin_ws/build/genmsg/build_env.sh /usr/bin/cmake /path/to/my_catkin_ws/src/genmsg --no-warn-unused-cli -DCATKIN_DEVEL_PREFIX=/path/to/my_catkin_ws/devel -DCMAKE_INSTALL_PREFIX=/path/to/my_catkin_ws/install`
    Output     << genmsg:cmake /path/to/my_catkin_ws/build/_logs/genmsg/build.cmake.000.log
    Not searching for unused variables given on the command line.
    Re-run cmake no build system arguments
    -- The C compiler identification is GNU
    -- The CXX compiler identification is GNU
    -- Check for working C compiler: /usr/bin/gcc
    -- Check for working C compiler: /usr/bin/gcc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Check for working CXX compiler: /usr/bin/c++
    -- Check for working CXX compiler: /usr/bin/c++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Using CATKIN_DEVEL_PREFIX: /path/to/my_catkin_ws/devel
    -- Using CMAKE_PREFIX_PATH: /path/to/my_catkin_ws/smach/devel;/opt/ros/hydro
    -- This workspace overlays: /path/to/my_catkin_ws/smach/devel;/opt/ros/hydro
    -- Found PythonInterp: /usr/bin/python (found version "2.7.3")
    -- Using PYTHON_EXECUTABLE: /usr/bin/python
    -- Python version: 2.7
    -- Using Debian Python package layout
    -- Using CATKIN_ENABLE_TESTING: ON
    -- Call enable_testing()
    -- Using CATKIN_TEST_RESULTS_DIR: /path/to/my_catkin_ws/build/genmsg/test_results
    -- Looking for include files CMAKE_HAVE_PTHREAD_H
    -- Looking for include files CMAKE_HAVE_PTHREAD_H - found
    -- Looking for pthread_create in pthreads
    -- Looking for pthread_create in pthreads - not found
    -- Looking for pthread_create in pthread
    -- Looking for pthread_create in pthread - found
    -- Found Threads: TRUE
    -- Found gtest sources under '/usr/src/gtest': gtests will be built
    -- catkin 0.5.90
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /path/to/my_catkin_ws/build/genmsg
    Finished   << genmsg:cmake
    Starting   >> genmsg:make
    Subprocess  > genmsg:make `cd /path/to/my_catkin_ws/build/genmsg && /usr/bin/make --jobserver-fds=3,4 -j`
    Output     << genmsg:make /path/to/my_catkin_ws/build/_logs/genmsg/build.make.000.log
    /usr/bin/cmake -H/path/to/my_catkin_ws/src/genmsg -B/path/to/my_catkin_ws/build/genmsg --check-build-system CMakeFiles/Makefile.cmake 0
    /usr/bin/cmake -E cmake_progress_start /path/to/my_catkin_ws/build/genmsg/CMakeFiles /path/to/my_catkin_ws/build/genmsg/CMakeFiles/progress.marks
    /usr/bin/make -f CMakeFiles/Makefile2 all
    make[1]: Entering directory `/path/to/my_catkin_ws/build/genmsg'
    make[1]: Nothing to be done for `all'.
    make[1]: Leaving directory `/path/to/my_catkin_ws/build/genmsg'
    /usr/bin/cmake -E cmake_progress_start /path/to/my_catkin_ws/build/genmsg/CMakeFiles 0
    Finished   << genmsg:make
    Finished  <<< genmsg        [ 2.0 seconds ]

Build Summary
-------------

At the end of each build, a brief build summary is printed to guarantee that
anomalies aren't missed. This summary displays the total runtime, the number
of successful jobs, the number of jobs which produced warnings, and the
number of jobs which weren't attempted due to failed dependencies.

.. code-block:: none

    [build] Runtime: 1.9 seconds total.
    [build] Summary: 4 of 7 jobs completed.
    [build]   Warnings:  None.
    [build]   Abandoned: 1 jobs were abandoned.
    [build]   Failed:    2 jobs failed.

A more detailed summary can also be printed, which lists the result for each
package in the workspace.

Building Subsets of Packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Consider a Catkin workspace with a **source space** populated with the following Catkin packages which have yet to be built:

.. code-block:: bash

    $ pwd
    /tmp/path/to/my_catkin_ws

    $ ls ./*
    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack


Building Specific Packages
--------------------------

In addition to the usage above, the ``--dry-run`` option will show what the behavior of ``catkin build`` will be with various other options.
For example, the following will happen when you specify a single package to build:

.. code-block:: bash

    $ catkin build roscpp_tutorials --dry-run
    ....
    Found '36' packages in 0.1 seconds.
    Packages to be built:
    - catkin               (catkin)
    - genmsg               (catkin)
    - gencpp               (catkin)
    - genlisp              (catkin)
    - genpy                (catkin)
    - console_bridge       (cmake)
    - cpp_common           (catkin)
    - message_generation   (catkin)
    - message_runtime      (catkin)
    - rosbuild             (catkin)
    - roscpp_traits        (catkin)
    - roslang              (catkin)
    - rospack              (catkin)
    - roslib               (catkin)
    - rostime              (catkin)
    - roscpp_serialization (catkin)
    - rosunit              (catkin)
    - rosconsole           (catkin)
    - std_msgs             (catkin)
    - rosgraph_msgs        (catkin)
    - xmlrpcpp             (catkin)
    - roscpp               (catkin)
    - roscpp_tutorials     (catkin)
    Total packages: 23

As shown above, only 23 packages (``roscpp_tutorials`` and its dependencies),
of the total 36 packages would be built.

Context-Aware Building
----------------------

In addition to building all packages or specified packages with various dependency requirements, ``catkin build`` can also determine the package containing the current working directory.
This is equivalent to specifying the name of the package on the command line, and is done by passing the ``--this`` option to ``catkin build`` like the following:

.. code-block:: bash

    $ cd /tmp/path/to/my_catkin_ws/src/roscpp_tutorials
    $ catkin build --this --dry-run
    ....
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    - roscpp_tutorials     (catkin)
    Total packages: 1

Skipping Packages
-----------------

Suppose you built every package up to ``roscpp_tutorials``, but that package had a build error.
After fixing the error, you could run the same build command again, but the ``build`` verb provides an option to save time in this situation.
If re-started from the beginning, none of the products of the dependencies of ``roscpp_tutorials`` would be re-built, but it would still take some time for the underlying byuildsystem to verify that for each package.

Those checks could be skipped, however, by jumping directly to a given package.
You could use the ``--start-with`` option to continue the build where you left off after fixing the problem. (The following example uses the ``--dry-run`` option again to preview the behavior):

.. code-block:: bash

    $ catkin build roscpp_tutorials --start-with roscpp_tutorials --dry-run
    ....
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    (skip) catkin               (catkin)
    (skip) genmsg               (catkin)
    (skip) gencpp               (catkin)
    (skip) genlisp              (catkin)
    (skip) genpy                (catkin)
    (skip) console_bridge       (cmake)
    (skip) cpp_common           (catkin)
    (skip) message_generation   (catkin)
    (skip) message_runtime      (catkin)
    (skip) rosbuild             (catkin)
    (skip) roscpp_traits        (catkin)
    (skip) roslang              (catkin)
    (skip) rospack              (catkin)
    (skip) roslib               (catkin)
    (skip) rostime              (catkin)
    (skip) roscpp_serialization (catkin)
    (skip) rosunit              (catkin)
    (skip) rosconsole           (catkin)
    (skip) std_msgs             (catkin)
    (skip) rosgraph_msgs        (catkin)
    (skip) xmlrpcpp             (catkin)
    (skip) roscpp               (catkin)
    ------ roscpp_tutorials     (catkin)
    Total packages: 23

However, you should be careful when using the ``--start-with`` option, as ``catkin build`` will assume that all dependencies leading up to that package have already been successfully built.

If you're only interested in building a *single* package in a workspace, you can also use the ``--no-deps`` option along with a package name.
This will skip all of the package's dependencies, build the given package, and then exit.

.. code-block:: bash

    $ catkin build roscpp_tutorials --no-deps roscpp_tutorials --dry-run
    ....
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    - roscpp_tutorials     (catkin)
    Total packages: 1

Building and Running Tests
^^^^^^^^^^^^^^^^^^^^^^^^^^

Running tests for a given package typically is done by invoking a special ``make`` target like ``test`` or ``run_tests``.
catkin packages all define the ``run_tests`` target which aggregates all types of tests and runs them together.
So in order to get tests to build and run for your packages you need to pass them this additional ``run_tests`` or ``test`` target as a command line option to ``make``.

To run catkin tests for all catkin packages in the workspace, use the following:

.. code-block:: bash

    $ catkin run_tests

Or the longer version:

.. code-block:: bash

    $ catkin build [...] --catkin-make-args run_tests

To run a catkin test for a specific catkin package, from a directory within that package:

.. code-block:: bash

    $ catkin run_tests --no-deps --this

For non-catkin packages which define a ``test`` target, you can do this:

.. code-block:: bash

    $ catkin build [...] --make-args test

If you want to run tests for just one package, then you should build that package and this narrow down the build to just that package with the additional make argument:

.. code-block:: bash

    $ # First build the package
    $ catkin build package
    ...
    $ # Then run its tests
    $ catkin build package --no-deps --catkin-make-args run_tests
    $ # Or for non-catkin packages
    $ catkin build package --no-deps --make-args test

For catkin packages and the ``run_tests`` target, failing tests will not result in an non-zero exit code.
So if you want to check for failing tests, use the ``catkin_test_results`` command like this:

.. code-block:: bash

    $ catkin_test_results build/<package name>

The result code will be non-zero unless all tests passed.


Advanced Options
^^^^^^^^^^^^^^^^

Temporarily Changing Build Flags
--------------------------------

While the build configuratoin flags are set and stored in the build context,
it's possible to temporarily override or augment them when using the ``build``
verb.

.. code-block:: bash

    $ catkin build --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter"

Building With Warnings
----------------------

It can sometimes be useful to compile with additional warnings enabled across your whole catkin workspace.
To achieve this, use a command similar to this:

.. code-block:: bash

    $ catkin build -v --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter"

This command passes the ``-DCMAKE_C_FLAGS=...`` arugment to all invocations of ``cmake``.


Configuring Build Jobs
----------------------

By default ``catkin build`` on a computer with ``N`` cores will build up to
``N`` packages in parallel and will distribute ``N`` ``make`` jobs among them
using an internal jobserver. If your platform doesn't support jobserver
scheduling, ``catkin build`` will pass ``-jN -lN`` to ``make`` for each package.

You can control the maximum number of packages allowed to build in parallel by
using the ``-p`` or ``--parallel-packages`` option and you can change the
number of ``make`` jobs available with the ``-j`` or ``--jobs`` option.

By default, these jobs options aren't passed to the underlying ``make``
command. To disable the jobserver, you can use the ``--no-jobserver`` option, and
you can pass flags directly to ``make`` with the ``--make-args`` option.

.. note::

    Jobs flags (``-jN`` and/or ``-lN``) can be passed directly to ``make`` by
    giving them to ``catkin build``, but other ``make`` arguments need to be
    passed to the ``--make-args`` option.


Configuring Memory Use
----------------------

In addition to CPU and load limits, ``catkin build`` can also limit the number of
running jobs based on the available memory, using the hidden ``--mem-limit`` flag.
This flag requires installing the Python ``psutil`` module and is useful on systems
without swap partitions or other situations where memory use needs to be limited.

Memory is specified either by percent or by the number of bytes.

For example, to specify that ``catkin build`` should not start additional parallel jobs
when 50% of the available memory is used, you could run:

.. code-block:: bash
  
    $ catkin build --mem-limit 50%

Alternatively, if it sohuld not start additional jobs when over 4GB of memory
is used, you can specifiy:

.. code-block:: bash
  
    $ catkin build --mem-limit 4G


Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin build [-h] [--workspace WORKSPACE] [--profile PROFILE]
                        [--dry-run] [--get-env PKGNAME] [--this] [--no-deps]
                        [--unbuilt] [--start-with PKGNAME | --start-with-this]
                        [--continue-on-failure] [--force-cmake] [--pre-clean]
                        [--no-install-lock] [--save-config] [-j JOBS]
                        [-p PACKAGE_JOBS] [--jobserver | --no-jobserver]
                        [--env-cache | --no-env-cache] [--cmake-args ARG [ARG ...]
                        | --no-cmake-args] [--make-args ARG [ARG ...] |
                        --no-make-args] [--catkin-make-args ARG [ARG ...] |
                        --no-catkin-make-args] [--verbose] [--interleave-output]
                        [--no-status] [--summarize] [--no-summarize]
                        [--override-build-tool-check]
                        [--limit-status-rate LIMIT_STATUS_RATE] [--no-notify]
                        [PKGNAME [PKGNAME ...]]

    Build one or more packages in a catkin workspace. This invokes `CMake`,
    `make`, and optionally `make install` for either all or the specified packages
    in a catkin workspace. Arguments passed to this verb can temporarily override
    persistent options stored in the catkin profile config. If you want to save
    these options, use the --save-config argument. To see the current config, use
    the `catkin config` command.

    optional arguments:
      -h, --help            show this help message and exit
      --workspace WORKSPACE, -w WORKSPACE
                            The path to the catkin_tools workspace or a directory
                            contained within it (default: ".")
      --profile PROFILE     The name of a config profile to use (default: active
                            profile)
      --dry-run, -n         List the packages which will be built with the given
                            arguments without building them.
      --get-env PKGNAME     Print the environment in which PKGNAME is built to
                            stdout.

    Packages:
      Control which packages get built.

      PKGNAME               Workspace packages to build, package dependencies are
                            built as well unless --no-deps is used. If no packages
                            are given, then all the packages are built.
      --this                Build the package containing the current working
                            directory.
      --no-deps             Only build specified packages, not their dependencies.
      --unbuilt             Build packages which have yet to be built.
      --start-with PKGNAME  Build a given package and those which depend on it,
                            skipping any before it.
      --start-with-this     Similar to --start-with, starting with the package
                            containing the current directory.
      --continue-on-failure, -c
                            Try to continue building packages whose dependencies
                            built successfully even if some other requested
                            packages fail to build.

    Build:
      Control the build behavior.

      --force-cmake         Runs cmake explicitly for each catkin package.
      --pre-clean           Runs `make clean` before building each package.
      --no-install-lock     Prevents serialization of the install steps, which is
                            on by default to prevent file install collisions

    Config:
      Parameters for the underlying build system.

      --save-config         Save any configuration options in this section for the
                            next build invocation.
      -j JOBS, --jobs JOBS  Maximum number of build jobs to be distributed across
                            active packages. (default is cpu count)
      -p PACKAGE_JOBS, --parallel-packages PACKAGE_JOBS
                            Maximum number of packages allowed to be built in
                            parallel (default is cpu count)
      --jobserver           Use the internal GNU Make job server which will limit
                            the number of Make jobs across all active packages.
      --no-jobserver        Disable the internal GNU Make job server, and use an
                            external one (like distcc, for example).
      --env-cache           Re-use cached environment variables when re-sourcing a
                            resultspace that has been loaded at a different stage
                            in the task.
      --no-env-cache        Don't cache environment variables when re-sourcing the
                            same resultspace.
      --cmake-args ARG [ARG ...]
                            Arbitrary arguments which are passes to CMake. It
                            collects all of following arguments until a "--" is
                            read.
      --no-cmake-args       Pass no additional arguments to CMake.
      --make-args ARG [ARG ...]
                            Arbitrary arguments which are passes to make.It
                            collects all of following arguments until a "--" is
                            read.
      --no-make-args        Pass no additional arguments to make (does not affect
                            --catkin-make-args).
      --catkin-make-args ARG [ARG ...]
                            Arbitrary arguments which are passes to make but only
                            for catkin packages.It collects all of following
                            arguments until a "--" is read.
      --no-catkin-make-args
                            Pass no additional arguments to make for catkin
                            packages (does not affect --make-args).

    Interface:
      The behavior of the command-line interface.

      --verbose, -v         Print output from commands in ordered blocks once the
                            command finishes.
      --interleave-output, -i
                            Prevents ordering of command output when multiple
                            commands are running at the same time.
      --no-status           Suppresses status line, useful in situations where
                            carriage return is not properly supported.
      --summarize, --summary, -s
                            Adds a build summary to the end of a build; defaults
                            to on with --continue-on-failure, off otherwise
      --no-summarize, --no-summary
                            Explicitly disable the end of build summary
      --override-build-tool-check
                            use to override failure due to using differnt build
                            tools on the same workspace.
      --limit-status-rate LIMIT_STATUS_RATE, --status-rate LIMIT_STATUS_RATE
                            Limit the update rate of the status bar to this
                            frequency. Zero means unlimited. Must be positive,
                            default is 10 Hz.
      --no-notify           Suppresses system pop-up notification.

