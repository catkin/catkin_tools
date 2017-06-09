``catkin config`` -- Configure a Workspace
==========================================

The ``config`` verb can be used to both view and manipulate a workspace's configuration options.
These options include all of the elements listed in the configuration summary.

By default, the ``config`` verb gets and sets options for a workspace's *active* profile.
If no profiles have been specified for a workspace, this is a default profile named ``default``.

.. note::

  Calling ``catkin config`` on an uninitialized workspace will not automatically   initialize it unless it is used with the ``--init`` option.

Viewing the Configuration Summary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once a workspace has been initialized, the configuration summary can be displayed by calling ``catkin config`` without arguments from anywhere under the root of the workspace.
Doing so will not modify your workspace.
The ``catkin`` command is context-sensitive, so it will determine which workspace contains the current working directory.

Appending or Removing List-Type Arguments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Several configuration options are actually *lists* of values.
Normally for these options, the given values will replace the current values in the configuration.

If you would only like to modify, but not replace the value of a list-type option, you can use the ``-a`` / ``--append-args`` and ``-r`` / ``--remove-args`` options to append or remove elements from these lists, respectively.

List-type options include:

 - ``--cmake-args``
 - ``--make-args``
 - ``--catkin-make-args``
 - ``--whitelist``
 - ``--blacklist``

Installing Packages
^^^^^^^^^^^^^^^^^^^

Without any additional arguments, packages are not "installed" using the standard CMake ``install()`` targets.
Addition of the ``--install`` option will configure a workspace so that it creates an **install space** and write the products of all install targets to that FHS tree.
The contents of the **install space**, which, by default, is located in a directory named ``install`` will look like the following:

.. code-block:: none

    $ ls ./install
    _setup_util.py bin            env.sh         etc            include
    lib            setup.bash     setup.sh       setup.zsh      share

Explicitly Specifying Workspace Chaining
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Normally, a catkin workspace automatically "extends" the other workspaces that have previously been sourced in your environment.
Each time you source a catkin setup file from a result-space (devel-space or install-space), it sets the ``$CMAKE_PREFIX_PATH`` in your environment, and this is used to build the next workspace.
This is also sometimes referred to as "workspace chaining" and sometimes the extended workspace is referred to as a "parent" workspace.

With ``catkin config``, you can explicitly set the workspace you want to extend, using the ``--extend`` argument.
This is equivalent to sourcing a setup file, building, and then reverting to the environment before sourcing the setup file.
For example, regardless of your current environment variable settings (like ``$CMAKE_PREFIX_PATH``), using ``--extend`` can build your workspace against the ``/opt/ros/indigo`` install space.

Note that in case the desired parent workspace is different from one already being used, using the ``--extend`` argument also necessitates cleaning your workspace with ``catkin clean``.

If you start with an empty ``CMAKE_PREFIX_PATH``, the configuration summary will show that you're not extending any other workspace, as shown below:

.. code-block:: bash

    $ echo $CMAKE_PREFIX_PATH

    $ mkdir -p /tmp/path/to/my_catkin_ws/src
    $ cd /tmp/path/to/my_catkin_ws
    $ catkin init
    --------------------------------------------------------------
    Profile:                     default
    Extending:                   None
    Workspace:                   /tmp/path/to/my_catkin_ws
    --------------------------------------------------------------
    Source Space:       [exists] /tmp/path/to/my_catkin_ws/src
    Log Space:          [exists] /tmp/path/to/my_catkin_ws/logs
    Build Space:        [exists] /tmp/path/to/my_catkin_ws/build
    Devel Space:        [exists] /tmp/path/to/my_catkin_ws/devel
    Install Space:      [unused] /tmp/path/to/my_catkin_ws/install
    DESTDIR:            [unused] None
    --------------------------------------------------------------
    Devel Space Layout:          linked
    Install Space Layout:        None
    --------------------------------------------------------------
    ...
    --------------------------------------------------------------
    Initialized new catkin workspace in `/tmp/path/to/my_catkin_ws`
    --------------------------------------------------------------

    --------------------------------------------------------------
    WARNING: Your workspace is not extending any other result
    space, but it is set to use a `linked` devel space layout.
    This requires the `catkin` CMake package in your source space
    in order to be built.
    --------------------------------------------------------------

At this point you have a workspace which doesn't extend anything.
With the default **devel space** layout, this won't build without the ``catkin`` CMake package, since this package is used to generate setup files.

If you realize this after the fact, you still can explicitly tell ``catkin build`` to extend  some result space.
Suppose you wanted to extend a standard ROS system install like ``/opt/ros/indigo``.
This can be done with the ``--extend`` option like so:

.. code-block:: bash

    $ catkin clean
    $ catkin config --extend /opt/ros/indigo
    --------------------------------------------------------------
    Profile:                     default
    Extending:        [explicit] /opt/ros/indigo
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
    Install Space Layout:        None
    --------------------------------------------------------------
    ...
    --------------------------------------------------------------
    Workspace configuration appears valid.
    --------------------------------------------------------------

    $ catkin build
    ...

    $ source devel/setup.bash
    $ echo $CMAKE_PREFIX_PATH
    /tmp/path/to/my_catkin_ws:/opt/ros/indigo


Whitelisting and Blacklisting Packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Packages can be added to a package *whitelist* or *blacklist* in order to change which packages get built.
If the *whitelist*  is non-empty, then a call to ``catkin build`` with no specific package names will only build the packages on the *whitelist*.
This means that you can still build packages not on the *whitelist*, but only if they are named explicitly or are dependencies of other whitelisted packages.

To set the whitelist, you can call the following command:

.. code-block:: text

    catkin config --whitelist foo bar

To clear the whitelist, you can use the ``--no-whitelist`` option:

.. code-block:: text

    catkin config --no-whitelist

If the *blacklist* is non-empty, it will filter the packages to be built in all cases except where a given package is named explicitly.
This means that blacklisted packages will not be built even if another package in the workspace depends on them.

.. note::

    Blacklisting a package does not remove it's build directory or build
    products, it only prevents it from being rebuilt.

To set the blacklist, you can call the following command:

.. code-block:: text

    catkin config --blacklist baz

To clear the blacklist, you can use the ``--no-blacklist`` option:

.. code-block:: text

    catkin config --no-blacklist

Note that you can still build packages on the blacklist and whitelist by passing their names to ``catkin build`` explicitly.

Accelerated Building with Environment Caching
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Each package is built in a special environment which is loaded from the current workspace and any workspaces that the current workspace is extending.
If you are confident that your workspace's environment is not changing during a build, you can tell ``catkin build`` to cache these environments with the ``--env-cache`` option.
This has the effect of dramatically reducing build times for workspaces where many packages are already built.


Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_config.txt
   :language: text
