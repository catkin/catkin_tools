Catkin Command Line Tools
=========================

This Python package provides command line tools for working with catkin and caktin workspaces.

Installing
----------

You can install the ``catkin_tools`` package as a binary through a package manager like ``pip`` or ``apt-get``, or from source.

Installing on Ubuntu with apt-get
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First you must have the ROS repositories which contain the ``.deb`` for ``catkin_tools``:

.. code-block:: bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

Once you have added that repository, run these commands to install ``catkin_tools``:

.. code-block:: bash

    $ sudo apt-get update
    $ sudo apt-get install python-catkin-tools

Installing on other platforms with pip
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Simply install it with ``pip``:

.. code-block:: bash

    $ sudo pip install -U catkin_tools

Installing from source
^^^^^^^^^^^^^^^^^^^^^^

First clone the source for ``catkin_tools``:

.. code-block:: bash

    $ git clone https://github.com/catkin/catkin_tools.git
    $ cd catkin_tools

Then install with the ``setup.py`` file:

.. code-block:: bash

    $ python setup.py install

Developing
----------

To setup ``catkin_tools`` for fast iteration during development, use the ``develop`` verb to ``setup.py``:

.. code-block:: bash

    $ python setup.py develop

Now the commands, like ``catkin``, will be in the system path and the local source files located in the ``catkin_tools`` folder will be on the ``PYTHONPATH``. When you are done with your development, undo this by running this command:

.. code-block:: bash

    $ python setup.py develop -u

Using the ``catkin`` command
----------------------------

The ``catkin`` Command-Line Interface (CLI) tool is the single point of entry for most of the tools provided by this package.
All invocations of the ``catkin`` CLI tool take this form:

.. code-block:: bash

    $ catkin [global options] <verb> [verb arguments and options]

The ``catkin`` CLI tool requires that you provide a verb.
The verbs could be many things, like ``build`` which builds a catkin workspace or ``list`` which simply lists the catkin packages found in one or more folders.
Optionally, global options can be provided before the verb, things like ``-d`` for debug level verbosity or ``-h`` for help on the ``catkin`` CLI tool itself.
Verbs can take arbitrary arguments and options, but they must all come after the verb.
For more help on a particular verb, simply pass ``-h`` or ``--help`` after the verb.

Using the ``catkin build`` command
----------------------------------

The ``build`` verb for the ``catkin`` command is used to build (configure and make) a catkin workspace.

Understanding a catkin workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A standard catkin workspace, as defined by `REP-0128 <http://www.ros.org/reps/rep-0128.html>`_, is a folder with a prescribed set "spaces", each in a folders within the workspace.

The ``source space`` is where the code for your packages resides and normally is in the folder ``/path/to/workspace/src``.
The ``source space`` is considered to be read-only, in that during a build no files or folders should be created in that folder.
Therefore catkin workspaces are said to be built "out of source", which simply means that the folder in which you build your code is not under or part of the folder with contains the source code.

Temporary build files are put into the "build space", which by default is in the ``/path/to/workspace/build`` folder.
The "build space" is the working directory in which commands like ``cmake`` and ``make`` are run.

Generated files, like executables, libraries, pkg-config files, CMake config files, or message code, are placed in the "devel space".
By convention the "devel space" is located as a peer to the "source space" and "build space" in the ``/path/to/workspace/devel`` folder.
The layout of the "devel space" is intended to mimic the root of a FHS filesystem, with folders like ``lib``, ``bin``, or ``share``.

In addition to the FHS folders, some setup scripts are generated in the "devel space", e.g. ``setup.bash`` or ``setup.zsh``.
These setup scripts are intended to make it easier to use the resulting "devel space" for building on top of the packages that were just built or for running programs built by those packages.
The setup script can be used like this in ``bash``:

.. code-block:: bash

    $ source /path/to/workspace/devel/setup.bash

Or like this in ``zsh``:

.. code-block:: zsh

    % source /path/to/workspace/devel/setup.zsh

``source``'ing these setup scripts adds this workspace and any "underlaid" workspaces to your environment, prefixing the ``CMAKE_PREFIX_PATH``, ``PKG_CONFIG_PATH``, ``PATH``, ``LD_LIBRARY_PATH``, ``CPATH``, and ``PYTHONPATH`` with local workspace folders.
The setup scripts will also execute any shell hooks exported by packages in the workspace, which is how ``roslib``, for example, sets the ``ROS_PACKAGE_PATH`` environment variable.

Finally, if the packages in the workspace are setup for installing, the install option can be invoked to install the
packages to the ``CMAKE_INSTALL_PREFIX``, a.k.a. the "install space".
The "install space", like the "devel space", has a FHS layout along with some generated setup files.
The "install space" is set to ``/path/to/workspace/install`` by changing the ``CMAKE_INSTALL_PREFIX`` by default.
This is done to prevent users from accidentally trying to install to the normal ``CMAKE_INSTALL_PREFIX`` path, ``/usr/local``.
Unlike the "devel space", the "install space" is completely stand alone and does not require the "source space" or "build space" to function, and is suitable for packaging.

.. note::

    Like the "devel space", the "install space" includes ``setup.*`` and related files at the top of the file hierarchy.
    This is not suitable for some packaging systems, so this can be disabled by passing the ``-DCATKIN_BUILD_BINARY_PACKAGE="1"`` option to ``cmake`` using the ``--cmake-args`` option for this verb.

Though there are conventions for the layout and location of the workspace's various "spaces", all of them can be changed using options to this verb.

Understanding the build process
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, a bit of history, there is a command called ``catkin_make`` which is provided by the ``catkin`` package and was designed to automated what we will call the merged build process.
The merged build process worked by adding all of the catkin packages in the workspace into one large CMake project and was configured with one invocation of ``cmake`` and built with one invocation of ``make``.
The command basically automated the standard CMake work flow while setting some sane defaults, essentially boiling down to these commands:

.. code-block:: bash

    $ mkdir build
    $ cd build
    $ cmake ../src -DCATKIN_DEVEL_SPACE=../devel -DCMAKE_INSTALL_PREFIX=../install
    $ make -j<number of cores> [optional target, e.g. install]

In the ``../src`` folder there would be a special "top-level" ``CMakeLists.txt`` which did the work of adding all the catkin projects below it to the single large CMake project.
The advantage of this is that the total configuration time is smaller than configuring each package in turn and that the Make targets can be parallelized even amongst dependent packages.
The disadvantage is that there is no fault isolation, i.e. an error in a leaf package will prevent all packages from configuring, or two packages might have colliding target names.

The other disadvantage is that this build process can only work on a homogeneous workspace of only catkin packages.
Other types of packages like plain CMake packages and autotools packages cannot be integrated into a single configuration and a single build step.
Because of this limitation the ``catkin_make_isolated`` command was created.
The ``catkin_make_isolated`` command used an isolated build process, where each package was configured, built, and the result sourced in turn.
This way each package is built in isolation and the next packages are built on the result of the current one, which also allows for automation of other work flows like the plain CMake work flow.
There were, however, some problems with ``catkin_make_isolated``, like the fact that you could not parallelize the building of packages which do not depend on each other and the fact that it was not robust to changes in the packages in the workspace.
These faults lead to the development of a parallel catkin make isolated, or ``pcmi``, as part of `Project Tango <http://osrfoundation.org/blog/project-tango-announced.html>`_.
``pcmi`` later became the ``catkin build`` command.

Therefore, the build process for this verb is an isolated build which can be parallelized and works by building each package in topological order, composing an environment for each package based on the packages on which it depends.
Other conceptual improvements over ``catkin_make_isolated`` include the ability to build part of a workspace, or robustly adapt a build when packages are added or removed from a workspace.

Understanding workspace packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A workspace's packages consist of any packages found in the "source space".
A package is any folder which contains a ``package.xml`` as defined in `REP-0127 <http://www.ros.org/reps/rep-0127.html>`_.
The ``catkin build`` command uses the ``build_depend``, ``run_depend``, and ``build_type`` tags in the ``package.xml``.
The ``*_depend`` tags are used to determine the topological build order of the packages.
The ``build_type`` tag is used to determine which build work flow to use on the package.
Packages without an explicitly defined ``build_type`` tag are assumed to be catkin packages, but plain CMake packages can be built by adding a ``package.xml`` file to the root of their source tree with the ``build_type`` flag set to ``cmake`` and appropriate ``build_depend`` and ``run_depend`` tags set.
This has been done for building packages like ``opencv``, ``pcl``, and ``flann`` in the past.

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
    Merge Develspaces:           False
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

In this example, we have setup a workspace with a few packages (actually its all the packages needed to build the `ros_tutorials` packages).
We start with only the "source space" and then use the ``--list`` option (short for ``--list-only``) to have the ``build`` verb figure out what packages it would build, and in what order, but then only list that information out and not actually build anything.

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

However, you should be careful when using the ``--start-with`` option, as ``catkin build`` will assume that all dependencies leading up to that package have been successfully built.

At this point the workspace has not been touched, but once we tell the ``build`` verb to actually build the workspace then a "build space" and a "devel space" will be created:

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
    Merge Develspaces:           False
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

    [cmi] Finished.
    [cmi] Runtime: 3 minutes and 54.6 seconds

    $ ls ./*
    ./build:
    catkin               genlisp              message_runtime      roscpp
    rosgraph_msgs        rosout               rostest              turtlesim
    cmi_logs             genmsg               ros_tutorials
    roscpp_serialization roslang              rospack              rostime
    xmlrpcpp             console_bridge       genpy                rosbuild
    roscpp_traits        roslaunch            rosparam             rosunit
    cpp_common           geometry_msgs        rosclean
    roscpp_tutorials     roslib               rospy                std_msgs
    gencpp               message_generation   rosconsole           rosgraph
    rosmaster            rospy_tutorials      std_srvs

    ./devel:
    catkin               genmsg               ros_tutorials
    roscpp_serialization roslang              rospack              rostime
    xmlrpcpp             console_bridge       genpy                rosbuild
    roscpp_traits        roslaunch            rosparam             rosunit
    cpp_common           geometry_msgs        rosclean
    roscpp_tutorials     roslib               rospy                std_msgs
    gencpp               message_generation   rosconsole           rosgraph
    rosmaster            rospy_tutorials      std_srvs             genlisp
    message_runtime      roscpp               rosgraph_msgs        rosout
    rostest              turtlesim

    ./src:
    catkin             console_bridge     genlisp            genpy
    message_runtime    ros_comm           roscpp_core        std_msgs
    common_msgs        gencpp             genmsg             message_generation
    ros                ros_tutorials      rospack

Since we didn't give any packages as arguments ``catkin build`` tried to build all of the packages in the workspace.
And as you can see, after the build finishes, we now have a "build space" with a folder for each package and a "devel space" which also has a folder for each package.
This would differ from ``catkin_make``, for example, which would have a combined "build space" and a single "devel space".

You may have noticed the status lines like this:

.. raw::

    [cmi - 5.9] [genmsg - 1.3] [message_runtime - 0.7] [ros_tutorials - 0.6] [rosbuild - 0.6]        [4/4 Active | 3/36 Completed]

This status line stays at the bottom of the screen and lets you know, at a glance, what the status of you build is.
The ``[cmi - 5.9]`` indicates that the total run time thus far has been ``5.9`` seconds.
The blocks like ``[genmsg - 1.3]`` means that you are currently building a package, in this case ``genmsg`` and it has been building for ``1.3`` seconds.
Justified to the right is the number of packages being actively built out of the total allowed in parallel and the number of completed packages out of the total, respectively, rendered like this: ``[4/4 Active | 3/36 Completed]``

Debugging with the ``catkin build`` command
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By default the output from each build is optimistically hidden to give a clean overview of the workspace build, but when there is a problem with a build a few things happen.
First, the package with a failure prints its build output to the screen between some enclosing lines:

.. raw::

    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/rospack'
    [ 66%] Built target rospack
    make[1]: *** [CMakeFiles/rosstackexe.dir/all] Interrupt: 2
    make[1]: *** [CMakeFiles/rospackexe.dir/all] Interrupt: 2
    make: *** [all] Interrupt: 2
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make -j4 -l4' failed with return code '-2'

And the status line is updated to reflect that that package has run into an issue by placing a ``!`` in front of it:

.. raw::

    [cmi - 1.7] [!cpp_common] [!rospack] [genlisp - 0.3]        [1/1 Active | 10/23 Completed]

Then ``catkin build`` command waits for the rest of the packages to finish (without starting new package builds) and then summarizes the errors for you:

.. raw::

    [cmi] There were errors:

    Failed to build package 'cpp_common' because the following command:

        # Command run in directory: /path/to/my_catkin_ws/build/cpp_common
        /path/to/my_catkin_ws/build/cpp_common/cmi_env.sh /usr/bin/make -j4 -l4

    Exited with return code: -2

    Failed to build package 'rospack' because the following command:

        # Command run in directory: /path/to/my_catkin_ws/build/rospack
        /path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make -j4 -l4

    Exited with return code: -2

If you don't want to scroll back up to find the error amongst the other output, you can ``cat`` the whole build log out of the ``cmi_logs`` folder in the "build space":

.. raw::

    $ cat build/cmi_logs/rospack.log
    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make cmake_check_build_system' in '/path/to/my_catkin_ws/build/rospack'
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make cmake_check_build_system' finished with return code '0'
    [rospack]: ==> '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make -j4 -l4' in '/path/to/my_catkin_ws/build/rospack'
    [ 66%] Built target rospack
    make[1]: *** [CMakeFiles/rosstackexe.dir/all] Interrupt: 2
    make[1]: *** [CMakeFiles/rospackexe.dir/all] Interrupt: 2
    make: *** [all] Interrupt: 2
    [rospack]: <== '/path/to/my_catkin_ws/build/rospack/cmi_env.sh /usr/bin/make -j4 -l4' failed with return code '-2'


