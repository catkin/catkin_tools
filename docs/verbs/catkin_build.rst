``catkin build`` -- Build Packages
==================================

The ``build`` verb for the ``catkin`` command is used to build one or more  packages in a catkin workspace.
Like most ``catkin`` verbs, the ``catkin build`` verb is context-aware. This means that it can be executed from within any directory contained by an *initialized* workspace.

If a workspace is not yet initialized, ``catkin build`` can initialize it, but only if it is called from the workspace root and the default workspace structure is desired.

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
Consider a Catkin workspace with a **source space** populated with the
following Catkin packages which have yet to be built:

.. code-block:: bash

    $ pwd
    /tmp/path/to/my_catkin_ws

    $ ls ./*
    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack

Previewing The Build
--------------------

Before actually building anything in the workspace, it is useful to preview which
packages ``catkin build`` will build, and in what order. This can be done with the
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

Building Specific Packages
--------------------------

In addition to the usage above, the ``--dry-run`` option will show what the
behavior of ``catkin build`` will be with various other options.
For example, the following will happen when you specify a single package to
build:

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

Skipping Packages
-----------------

Suppose you built every package up to ``roscpp_tutorials``, but that package
had a build error.
After fixing the error, you could run the same build command again, but the
``build`` verb provides an option to save time in this situation.
If re-started from the beginning, none of the products of the dependencies of
``roscpp_tutorials`` would be re-built, but it would still take some time for
the underlying byuildsystem to verify that for each package.

Those checks could be skipped, however, by jumping directly to a given package.
You could use the ``--start-with`` option to continue the build where you left
off after fixing the problem. (The following example uses the ``--dry-run``
option again to preview the behavior):

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

However, you should be careful when using the ``--start-with`` option, as
``catkin build`` will assume that all dependencies leading up to that package
have already been successfully built.

If you're only interested in building a *single* package in a workspace, you
can also use the ``--no-deps`` option along with a package name. This will
skip all of the package's depdendencies, build the given package, and then exit.

.. code-block:: bash

    $ catkin build roscpp_tutorials --no-deps roscpp_tutorials --dry-run
    ....
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    - roscpp_tutorials     (catkin)
    Total packages: 1

Build Products
--------------

At this point the workspace has not been modified, but once we tell the
``build`` verb to actually build the workspace then directories for a **build
space** and a **devel space** will be created:

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
    Workspace configuration appears valid.
    --------------------------------------------------------------
    Found '36' packages in 0.0 seconds.
    Starting ==> catkin
    Starting ==> console_bridge
    Finished <== catkin [ 2.4 seconds ]

    ....

    [build] Finished.
    [build] Runtime: 3 minutes and 54.6 seconds

Since no packages were given as arguments, ``catkin build`` built all of
the packages in the workspace.

As shown above, after the build finishes, we now have a **build space** with a
folder containing the intermediate build products for each package and a
**devel space** with an FHS layout into which all the final build products have
been written.

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
    ``catkin_make_isolated`` which would have an insolated FHS directory for
    each package in the **devel space**.

Context-Aware Building
^^^^^^^^^^^^^^^^^^^^^^

In addition to building all packages or specified packages with various dependency requirements,
``catkin build`` can also determine the package containing the current working directory. This
is equivalent to specifying the name of the package on the command line, and is
done by passing the ``--this`` option to ``catkin build`` like the following:

.. code-block:: bash

    $ cd /tmp/path/to/my_catkin_ws/src/roscpp_tutorials
    $ catkin build --this --dry-run
    ....
    Found '36' packages in 0.0 seconds.
    Packages to be built:
    - roscpp_tutorials     (catkin)
    Total packages: 1

Controlling Command-Line Output
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Status Line
-----------

While running ``catkin build`` with default options, you would have seen the
"live" status lines similar to the following:

.. code-block:: none

    [build - 5.9] [genmsg - 1.3] [message_runtime - 0.7] ...        [4/4 Active | 3/36 Completed]

This status line stays at the bottom of the screen and displays the continuously-updated progress
of the entire build as well as the active build jobs which are still running. It is composed
of the following information:

 * **Total Build Time** -- The first block on the left, indicates the total
   elapsed build time in seconds thus far.  Above, ``[build - 5.9]`` means that
   the build has been running for a total of ``5.9`` seconds.
 * **Active Job Status** -- The next blocks show the currently active jobs with as
   name of the package being built and the elapsed time for that job, in
   seconds.  The above block like ``[genmsg - 1.3]`` means that the ``genmsg``
   package is currently being built, and it has been building for ``1.3``
   seconds.
 * **Active and Completed Counts** -- The final block, justified to the right,
   is the number of packages being actively built out of the total allowed
   parallel jobs (specified with the ``-p`` options) as well as the number of
   completed packages out of the total. Above, the block ``[4/4 Active | 3/36
   Completed]`` means that there are four out of four jobs active and three of
   the total 36 packages to be built have been completed.

This status line can be disabled by passing the ``--no-status`` option to ``catkin build``.

Package Build Messages
----------------------

Normally, unless an error occurs, the output from each package's build proces
is collected but not printed to the console. All that is printed is a pair of
messages designating the start and end of a package's build. This is formatted
like the following for the ``genmsg`` package:

.. code-block:: none

    Starting ==> genmsg
    Finished <== genmsg [ 2.4 seconds ]

However, if you would like to see more of the messages from the underlying
buildsystem, you can invoke the ``-v`` or ``--verbose`` option.
This will print the normal message when a package build starts and finished as
well as the output of each build command in a block, once it finishes:

.. code-block:: none

    Starting ==> catkin

    [catkin]: ==> '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/local/bin/cmake /path/to/my_catkin_ws/src/catkin -DCATKIN_DEVEL_PREFIX=/path/to/my_catkin_ws/devel/catkin -DCMAKE_INSTALL_PREFIX=/path/to/my_catkin_ws/install' in '/path/to/my_catkin_ws/build/catkin'
    -- The C compiler identification is Clang 5.0.0
    -- The CXX compiler identification is Clang 5.0.0
    -- Check for working C compiler: /usr/bin/cc
    -- Check for working C compiler: /usr/bin/cc -- works
    -- Detecting C compiler ABI info
    -- Detecting C compiler ABI info - done
    -- Check for working CXX compiler: /usr/bin/c++
    -- Check for working CXX compiler: /usr/bin/c++ -- works
    -- Detecting CXX compiler ABI info
    -- Detecting CXX compiler ABI info - done
    -- Using CATKIN_DEVEL_PREFIX: /path/to/my_catkin_ws/devel/catkin
    -- Using CMAKE_PREFIX_PATH: /path/to/my_catkin_ws/install
    -- This workspace overlays: /path/to/my_catkin_ws/install
    -- Found PythonInterp: /usr/bin/python (found version "2.7.5")
    -- Using PYTHON_EXECUTABLE: /usr/bin/python
    -- Python version: 2.7
    -- Using default Python package layout
    -- Found PY_em: /Library/Python/2.7/site-packages/em.pyc
    -- Using CATKIN_ENABLE_TESTING: ON
    -- Call enable_testing()
    -- Using CATKIN_TEST_RESULTS_DIR: /path/to/my_catkin_ws/build/catkin/test_results
    -- Found gtest: gtests will be built
    -- catkin 0.5.86
    -- Configuring done
    -- Generating done
    -- Build files have been written to: /path/to/my_catkin_ws/build/catkin
    [catkin]: <== '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/local/bin/cmake /path/to/my_catkin_ws/src/catkin -DCATKIN_DEVEL_PREFIX=/path/to/my_catkin_ws/devel/catkin -DCMAKE_INSTALL_PREFIX=/path/to/my_catkin_ws/install' finished with return code '0'

    [catkin]: ==> '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/catkin'
    [catkin]: <== '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/bin/make -j4 -l4' finished with return code '0'

    [catkin]: ==> '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/bin/make install' in '/path/to/my_catkin_ws/build/catkin'
    Install the project...
    -- Install configuration: ""
    ... truncated for brevity
    [catkin]: <== '/path/to/my_catkin_ws/build/catkin/build_env.sh /usr/bin/make install' finished with return code '0'

    Finished <== catkin [ 3.4 seconds ]

.. note::

    The printing of these command outputs maybe be interleaved with commands
    from other package builds if more than one package is being built at the
    same time.

By default ``catkin build`` will build up to ``N`` packages in parallel and
pass ``-jN -lN`` to ``make`` where ``N`` is the number of cores in your
computer.

You can change the number of packages allowed to build in parallel
by using the ``-p`` or ``--parallel-jobs`` option and you can change the jobs
flags given to ``make`` by passing them directly to ``catkin build``, i.e.
``catkin build -j1`` will result in ``make -j1 ...`` getting called to build
the packages.

.. note::

    Jobs flags (``-jN`` and/or ``-lN``) can be passed directly to ``make`` by
    giving them to ``catkin build``, but other ``make`` arguments need to be
    passed to the ``--make-args`` option.

If you want to see the output from commands streaming to the screen, then you
can use the ``-i`` or ``--interleave`` option.  This option will cause the
output from commands to be pushed to the screen immediately, instead of
buffering until the command finishes.  This ends up being pretty confusing, so
when interleaved output is used ``catkin build`` prefixes each line with
``[<package name>]:`` like this:

.. code-block:: none

    [roscpp_traits]: ==> '/Users/william/my_catkin_ws/build/roscpp_traits/build_env.sh /usr/bin/make cmake_check_build_system' in '/Users/william/my_catkin_ws/build/roscpp_traits'
    [ros_tutorials]: -- The CXX compiler identification is Clang 5.0.0
    [ros_tutorials]: -- Check for working C compiler: /usr/bin/cc
    [roscpp_traits]: ==> '/Users/william/my_catkin_ws/build/roscpp_traits/build_env.sh /usr/bin/make -j4 -l4' in '/Users/william/my_catkin_ws/build/roscpp_traits'
    [rosbuild]: ==> '/Users/william/my_catkin_ws/build/rosbuild/build_env.sh /usr/bin/make -j4 -l4' in '/Users/william/my_catkin_ws/build/rosbuild'
    [rosclean]: -- The C compiler identification is Clang 5.0.0
    [ros_tutorials]: -- Check for working C compiler: /usr/bin/cc -- works
    [ros_tutorials]: -- Detecting C compiler ABI info
    [rosclean]: -- The CXX compiler identification is Clang 5.0.0
    [rosclean]: -- Check for working C compiler: /usr/bin/cc

.. note::

    When you use ``-p 1`` and ``-v`` at the same time, ``-i`` is implicitly added.


Running Tests Built in a Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Running tests for a given package typically is done by invoking a special ``make`` target like ``test`` or ``run_tests``.
catkin packages all define the ``run_tests`` target which aggregates all types of tests and runs them together.
So in order to get tests to build and run for your packages you need to pass them this additional ``run_tests`` or ``test`` target as a command line option to ``make``.

To run catkin tests for catkin packages, use the following:

.. code-block:: bash

    $ catkin build [...] --catkin-make-args run_tests

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

Debugging Build Errors
^^^^^^^^^^^^^^^^^^^^^^

As mentioned above, by default the output from each build is optimistically
hidden to give a clean overview of the workspace build, but when there is a
problem with a build a few things happen.

First, the package with a failure prints the failing command's output to the
screen between some enclosing lines:

.. code-block:: none

    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/rospack'
    [ 66%] Built target rospack
    make[1]: *** [CMakeFiles/rosstackexe.dir/all] Interrupt: 2
    make[1]: *** [CMakeFiles/rospackexe.dir/all] Interrupt: 2
    make: *** [all] Interrupt: 2
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' failed with return code '-2'

And the status line is updated to reflect that that package has run into an
issue by placing a ``!`` in front of it:

.. code-block:: none

    [build - 1.7] [!cpp_common] [!rospack] [genlisp - 0.3]        [1/1 Active | 10/23 Completed]

Then the ``catkin build`` command waits for the rest of the packages to finish
(without starting new package builds) and then summarizes the errors for you:

.. code-block:: none

    [build] There were errors:

    Failed to build package 'cpp_common' because the following command:

        # Command run in directory: /path/to/my_catkin_ws/build/cpp_common
        /path/to/my_catkin_ws/build/cpp_common/build_env.sh /usr/bin/make -j4 -l4

    Exited with return code: -2

    Failed to build package 'rospack' because the following command:

        # Command run in directory: /path/to/my_catkin_ws/build/rospack
        /path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4

    Exited with return code: -2

If you don't want to scroll back up to find the error amongst the other output,
you can ``cat`` the whole build log out of the ``build_logs`` folder in the
**build space**:

.. code-block:: bash

    $ cat build/build_logs/rospack.log
    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make cmake_check_build_system' in '/path/to/my_catkin_ws/build/rospack'
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make cmake_check_build_system' finished with return code '0'
    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/rospack'
    [ 66%] Built target rospack
    make[1]: *** [CMakeFiles/rosstackexe.dir/all] Interrupt: 2
    make[1]: *** [CMakeFiles/rospackexe.dir/all] Interrupt: 2
    make: *** [all] Interrupt: 2
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' failed with return code '-2'

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin build [-h] [--workspace WORKSPACE] [--profile PROFILE]
                        [--list-only] [--this] [--no-deps]
                        [--start-with PKGNAME | --start-with-this] [--force-cmake]
                        [--no-install-lock] [--save-config]
                        [--parallel-jobs PARALLEL_JOBS]
                        [--cmake-args ARG [ARG ...] | --no-cmake-args]
                        [--make-args ARG [ARG ...] | --no-make-args]
                        [--catkin-make-args ARG [ARG ...] | --no-catkin-make-args]
                        [--force-color] [--verbose] [--interleave-output]
                        [--no-status] [--no-notify]
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

    Packages:
      Control which packages get built.

      PKGNAME               Workspace packages to build, package dependencies are
                            built as well unless --no-deps is used. If no packages
                            are given, then all the packages are built.
      --this                Build the package containing the current working
                            directory.
      --no-deps             Only build specified packages, not their dependencies.
      --start-with PKGNAME  Build a given package and those which depend on it,
                            skipping any before it.
      --start-with-this     Similar to --start-with, starting with the package
                            containing the current directory.

    Build:
      Control the build behaiovr.

      --force-cmake         Runs cmake explicitly for each catkin package.
      --no-install-lock     Prevents serialization of the install steps, which is
                            on by default to prevent file install collisions

    Config:
      Parameters for the underlying buildsystem.

      --save-config         Save any configuration options in this section for the
                            next build invocation.
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

    Interface:
      The behavior of the command-line interface.

      --force-color         Forces catkin build to ouput in color, even when the
                            terminal does not appear to support it.
      --verbose, -v         Print output from commands in ordered blocks once the
                            command finishes.
      --interleave-output, -i
                            Prevents ordering of command output when multiple
                            commands are running at the same time.
      --no-status           Suppresses status line, useful in situations where
                            carriage return is not properly supported.
      --no-notify           Suppresses system popup notification.


