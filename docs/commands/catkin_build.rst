The ``catkin build`` command
----------------------------

The ``build`` verb for the ``catkin`` command is used to build (configure and make) a catkin workspace.

Understanding a catkin workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A standard catkin workspace, as defined by `REP-0128 <http://www.ros.org/reps/rep-0128.html>`_, is a folder with a prescribed set "spaces", each in a folders within the workspace.

The ``source space`` is where the code for your packages resides and normally is in the folder ``/path/to/workspace/src``.
The build command considers ``source space`` to be read-only, in that during a build no files or folders should be created or modified in that folder.
Therefore catkin workspaces are said to be built "out of source", which simply means that the folder in which you build your code is not under or part of the folder with contains the source code.

Temporary build files are put into the ``build space``, which by default is in the ``/path/to/workspace/build`` folder.
The ``build space`` is the working directory in which commands like ``cmake`` and ``make`` are run.

Generated files, like executables, libraries, pkg-config files, CMake config files, or message code, are placed in the "devel space".
By convention the ``devel space`` is located as a peer to the ``source space`` and ``build space`` in the ``/path/to/workspace/devel`` folder.
The layout of the ``devel space`` is intended to mimic the root of a `FHS filesystem <https://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_, with folders like ``include``, ``lib``, ``bin``, and ``share``.

In addition to the FHS folders, some setup scripts are generated in the ``devel space``, e.g. ``setup.bash`` or ``setup.zsh``.
These setup scripts are intended to make it easier to use the resulting ``devel space`` for building on top of the packages that were just built or for running programs built by those packages.
The setup script can be used like this in ``bash``:

.. code-block:: bash

    $ source /path/to/workspace/devel/setup.bash

Or like this in ``zsh``:

.. code-block:: bash

    % source /path/to/workspace/devel/setup.zsh

``source``'ing these setup scripts adds this workspace and any "underlaid" workspaces to your environment, prefixing the ``CMAKE_PREFIX_PATH``, ``PKG_CONFIG_PATH``, ``PATH``, ``LD_LIBRARY_PATH``, ``CPATH``, and ``PYTHONPATH`` with local workspace folders.
The setup scripts will also execute any shell hooks exported by packages in the workspace, which is how ``roslib``, for example, sets the ``ROS_PACKAGE_PATH`` environment variable.

Finally, if the packages in the workspace are setup for installing, the ``--install`` option can be invoked to install the packages to the ``CMAKE_INSTALL_PREFIX``, which in `REP-0128 <http://www.ros.org/reps/rep-0128.html>`_ terms is the "install space".
The "install space", like the "devel space", has a FHS layout along with some generated setup files.
The "install space" is set to ``/path/to/workspace/install`` by changing the ``CMAKE_INSTALL_PREFIX`` by default.
This is done to prevent users from accidentally trying to install to the normal ``CMAKE_INSTALL_PREFIX`` path, ``/usr/local``.
Unlike the "devel space", the "install space" is completely stand alone and does not require the "source space" or "build space" to function, and is suitable for packaging.

.. note::

    Like the "devel space", the "install space" includes ``setup.*`` and related files at the top of the file hierarchy.
    This is not suitable for some packaging systems, so this can be disabled by passing the ``-DCATKIN_BUILD_BINARY_PACKAGE="1"`` option to ``cmake`` using the ``--cmake-args`` option for this verb.
    Though this will suppress the installation of the setup files, you will loose the functionality provided by them, namely extending the environment and executing environment hooks.

Though there are conventions for the layout and location of the workspace's various "spaces", all of the locations can be changed using options to this verb.

Understanding the build process
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, a bit of history, there is a command called ``catkin_make`` which is provided by the ``catkin`` package and was designed to automated what we will call the "merged" build process.
The merged build process worked by adding all of the catkin packages in the workspace into one large CMake project and was configured with one invocation of ``cmake`` and built with one invocation of ``make``.
The command basically automated the standard CMake work flow while setting some sane defaults, essentially boiling down to these commands:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src -DCATKIN_DEVEL_SPACE=../devel -DCMAKE_INSTALL_PREFIX=../install
    $ make -j<number of cores> -l<number of cores> [optional target, e.g. install]

In the ``../src`` folder there would be a boiler-plate "top-level" ``CMakeLists.txt`` which did the work of adding all the catkin projects below it to the single large CMake project.
The advantage of this is that the total configuration time is smaller than configuring each package in turn and that the Make targets can be parallelized even amongst dependent packages.
The disadvantage is that there is no fault isolation, e.g. an error in a leaf package will prevent all packages from configuring, or two packages might have colliding target names. The merged build process can even cause CMake errors to go undetected, and can depend on the order in which independent packages are built. 

Another disadvantage of the merged build process is that it can only work on a homogeneous workspace consisting only of catkin packages.
Other types of packages like plain CMake packages and autotools packages cannot be integrated into a single configuration and a single build step.
Because of this limitation the ``catkin_make_isolated`` command was created.
The ``catkin_make_isolated`` command uses an isolated build process, wherein each package is configured, built, and the results sourced in turn.
This way each package is built in isolation and the next packages are built on the result of the current one, which also allows for automation of other work flows like the plain CMake work flow.
There are, however, some problems with ``catkin_make_isolated``, including the inability to parallelize the building of packages which do not depend on each other and a lack of robustness to changes in the list of packages in the workspace.
These limitations lead to the development of a parallel version of catkin make isolated, or ``pcmi``, as part of `Project Tango <http://osrfoundation.org/blog/project-tango-announced.html>`_.
``pcmi`` later became the ``build`` verb of the ``catkin`` command.

Therefore, the principle behavior of the ``build`` verb is to build each package in isolation and in topological order; composing an environment for each package's build based on the packages on which it depends.
Other conceptual improvements over ``catkin_make_isolated`` include the ability to build part of a workspace, or robustly adapt a build when packages are added to or removed from a workspace. See the following sections for other features specific to this verb.

Understanding workspace packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A workspace's packages consist of any packages found in the ``source space``.
A package is any folder which contains a ``package.xml`` as defined in `REP-0127 <http://www.ros.org/reps/rep-0127.html>`_.
The ``catkin build`` command uses the ``depend``, ``build_depend``, ``run_depend``, and ``build_type`` tags in the ``package.xml``.
The ``*_depend`` tags are used to determine the topological build order of the packages.
The ``build_type`` tag is used to determine which build work flow to use on the package.
Packages without an explicitly defined ``build_type`` tag are assumed to be catkin packages, but plain CMake packages can be built by adding a ``package.xml`` file to the root of their source tree with the ``build_type`` flag set to ``cmake`` and appropriate ``build_depend`` and ``run_depend`` tags set, as described in `REP-0136 <http://www.ros.org/reps/rep-0136.html>`_.
This has been done in the past for building packages like ``opencv``, ``pcl``, and ``flann``.

Typical ``catkin build`` command usage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The typical work flow for using ``catkin build`` is to execute it in the root of a catkin workspace:

.. code-block:: bash

    $ pwd
    /path/to/my_catkin_ws

    $ ls ./*
    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack

    $ catkin build --list
    ----------------------------------------------------------------
    Workspace:                   /path/to/my_catkin_ws
    Buildspace:                  /path/to/my_catkin_ws/build
    Develspace:                  /path/to/my_catkin_ws/devel
    Installspace:                /path/to/my_catkin_ws/install
    DESTDIR:                     None
    ----------------------------------------------------------------
    Isolate Develspaces:           False
    Install Packages:            False
    Isolate Installs:            False
    ----------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    ----------------------------------------------------------------
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

In this example, we have setup a workspace with a few packages (actually its all the packages needed to build the ``ros_tutorials``).
We start with only the ``source space`` and then use the ``--list`` option (short for ``--list-only``) to have the ``build`` verb figure out what packages it would build, and in what order, but then only list that information out and not actually build anything.

You can use the ``--list`` option to preview the behavior of ``catkin build`` will be with various options.
For example, you can see what will happen when you specify a single package to build:

.. code-block:: bash

    $ catkin build roscpp_tutorials --list
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

As you can see, only 23 packages (``roscpp_tutorials`` and its dependencies), of the total 36 packages will be built.

Lets say you built every package up to ``roscpp_tutorials``, but that package had a build error, and you want to jump directly to it.
You could use the ``--start-with`` option along with the ``--list`` option to preview the result:

.. code-block:: bash

    $ catkin build roscpp_tutorials --start-with roscpp_tutorials --list
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

At this point the workspace has not been modified, but once we tell the ``build`` verb to actually build the workspace then directories for a ``build space`` and a ``devel space`` will be created:

.. code-block:: bash

    $ catkin build
    Creating buildspace directory, '/path/to/my_catkin_ws/build'
    ----------------------------------------------------------------
    Workspace:                   /path/to/my_catkin_ws
    Buildspace:                  /path/to/my_catkin_ws/build
    Develspace:                  /path/to/my_catkin_ws/devel
    Installspace:                /path/to/my_catkin_ws/install
    DESTDIR:                     None
    ----------------------------------------------------------------
    Isolate Develspaces:         False
    Install Packages:            False
    Isolate Installs:            False
    ----------------------------------------------------------------
    Additional CMake Args:       None
    Additional Make Args:        None
    Additional catkin Make Args: None
    ----------------------------------------------------------------
    Found '36' packages in 0.0 seconds.
    Starting ==> catkin
    Starting ==> console_bridge
    Finished <== catkin [ 2.4 seconds ]

    ... build goes on

    [build] Finished.
    [build] Runtime: 3 minutes and 54.6 seconds

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

Since we didn't give any packages as arguments ``catkin build`` tried to build all of the packages in the workspace.
And as you can see, after the build finishes, we now have a ``build space`` with a folder for each package and a ``devel space`` with an FHS layout into which all the build products have been written.
This differs from the behavior of ``catkin_make``, for example, which would have all of the build files and intermediate build products in a combined ``build space`` or ``catkin_make_isolated`` which would have an insolated FHS directory for each package in the ``devel space``.

Without any additional arguments, the packages are not installed.
If we provide ``catkin build`` with the ``--install`` option, an ``install space`` will be created containing the results of the install targets from all of the built packages in an aditional FHS tree. The contents of the ``install space``, which, by default, is located in a directory named ``install`` will look like the following:

.. code-block:: none

    $ ls ./install
    _setup_util.py bin            env.sh         etc            include
    lib            setup.bash     setup.sh       setup.zsh      share

Controlling output of ``catkin build``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You may have noticed the status lines like this:

.. code-block:: none

    [build - 5.9] [genmsg - 1.3] [message_runtime - 0.7] ...        [4/4 Active | 3/36 Completed]

This status line stays at the bottom of the screen and lets you know, at a glance, what the status of you build is.
The ``[build - 5.9]`` indicates that the total run time thus far has been ``5.9`` seconds.
The blocks like ``[genmsg - 1.3]`` means that you are currently building a package, in this case ``genmsg``, and it has been building for ``1.3`` seconds.
Justified to the right is the number of packages being actively built out of the total allowed in parallel and the number of completed packages out of the total, respectively, rendered like this: ``[4/4 Active | 3/36 Completed]``

This status line can be disabled by passing the ``--no-status`` option to ``catkin build``.

Normally the output from each build is collected and not printed, unless there is an error, and a pair of messages are the only thing printed to signify the start and end of a package's build:

.. code-block:: none

    Starting ==> catkin
    Finished <== catkin [ 2.4 seconds ]

However, if you would like to see more than this you can invoke the ``-v`` or ``--verbose`` option.
This will give a message when a package build starts and finished as well as printing the output of each build command in a block, once it finishes:

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

The printing of these command outputs maybe be interleaved with commands from other package builds if more than one package is being built at the same time.

By default ``catkin build`` will build up to ``N`` packages in parallel and pass ``-jN -lN`` to ``make`` where ``N`` is the number of cores in your computer.
You can change the number of packages allowed to build in parallel by using the ``-p`` or ``--parallel-jobs`` option and you can change the jobs flags given to ``make`` by passing them directly to ``catkin build``, i.e. ``catkin build -j1`` will result in ``make -j1 ...`` getting called to build the packages.

.. note::

    Jobs flags (``-jN`` and/or ``-lN``) can be passed directly to ``make`` by giving them to ``catkin build``, but other ``make`` arguments need to be passed to the ``--make-args`` option.

If you want to see the output from commands streaming to the screen, then you can use the ``-i`` or ``--interleave`` option.
This option will cause the output from commands to be pushed to the screen immediately, instead of buffering until the command finishes.
This ends up being pretty confusing, so when interleaved output is used ``catkin build`` prefixes each line with ``[<package name>]: `` like this:

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

When you use ``-p 1`` and ``-v`` at the same time, ``-i`` is implicitly added.

Debugging with ``catkin build``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default the output from each build is optimistically hidden to give a clean overview of the workspace build, but when there is a problem with a build a few things happen.

First, the package with a failure prints the failing command's output to the screen between some enclosing lines:

.. code-block:: none

    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/rospack'
    [ 66%] Built target rospack
    make[1]: *** [CMakeFiles/rosstackexe.dir/all] Interrupt: 2
    make[1]: *** [CMakeFiles/rospackexe.dir/all] Interrupt: 2
    make: *** [all] Interrupt: 2
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/build_env.sh /usr/bin/make -j4 -l4' failed with return code '-2'

And the status line is updated to reflect that that package has run into an issue by placing a ``!`` in front of it:

.. code-block:: none

    [build - 1.7] [!cpp_common] [!rospack] [genlisp - 0.3]        [1/1 Active | 10/23 Completed]

Then the ``catkin build`` command waits for the rest of the packages to finish (without starting new package builds) and then summarizes the errors for you:

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

If you don't want to scroll back up to find the error amongst the other output, you can ``cat`` the whole build log out of the ``build_logs`` folder in the ``build space``:

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
