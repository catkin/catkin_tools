Configuration Summary
=====================

Contents of the Config Summary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Most ``catkin`` comands which modify the a workspace's configuration will
display the standard configuration summary, as shown below:

.. code-block:: bash

    $ cd /tmp/path/to/my_catkin_ws
    $ catkin config
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

This summary describes the layout of the workspace as well as other important
settings which influence build and execution behavior. Each of these options
can be modified either with the ``config`` verb's options described in the full
command-line usage or by changing enviornment variables. The summary is
composed of the following sections:

Overview Section
----------------

- **Profile** -- The name of this configuration
- **Extending** -- Describes if your current configuration will extend another Catkin workspace, and thorugh which mechanism it determined the location of the extended workspace:

  - *No Chaining*

    .. code-block:: bash

          Extending:                   None

  - *Implicit Chaining* -- Derived from the ``CMAKE_PREFIX_PATH`` environment or cache variable.

    .. code-block:: bash

          Extending:             [env] /opt/ros/hydro
    .. code-block:: bash

          Extending:          [cached] /opt/ros/hydro

  - *Explicit Chaining* -- Specified by ``catkin config --extend``

    .. code-block:: bash

          Extending:        [explicit] /opt/ros/hydro

- **[* Space]** -- Lists the paths to each of the catkin "spaces" and whether or not they exist
- **DESTDIR** -- An optional prefix to the **install space** as defined by `GNU Standards <https://www.gnu.org/prep/standards/html_node/DESTDIR.html/>`_
- **Isolate Develspaces** -- Builds products (like libraries and binaries) into individual FHS subdirectories in the **devel space**, instead of a single FHS directory
- **Install Packages** -- Enable creating and installation into the **install space**
- **Isolate Installs** -- Installs products into individual FHS subdirectories in the **install space**
- **Additional CMake Args** -- Arguments to be passed to CMake during the *configuration* step for all packages to be built.
- **Additional Make Args** -- Arguments to be passed to Make during the *build* step for all packages to be built.
- **Additional catkin Make Args** -- Similar to **Additional Make Args** but only applies to Catkin packages.

Notes Section
-------------

The summary will sometimes contain notes about the workspace or the action that
you're performing, or simply tell you that the workspace configuration appears
valid.

Warnings Section
----------------

If something is wrong with your configuration such as a missing source space,
an additional secion will appear at the bottom of the summary with details on
what is wrong and how you can fix it.

Workspace Chaining Mode
^^^^^^^^^^^^^^^^^^^^^^^

An important property listed in the configuration configuration which deserves
attention is the summary value of the ``Extending`` property. This affects
which other collections of libraries and packages which will be visible to your
workspace.  This is process called "workspace chaining." For more details on this
see the details about workspace chaining and ``CMAKE_PREFIX_PATH`` in
:doc:`Workspace Mechanics <mechanics>`.

The information about which workspace to extend can come from a few different
sources, and can be classified in one of three ways:

No Chaining
-----------

This is what is shown in the above example configuration and it implies that
there are no other Catkin workspaces which this workspace extends. The user has
neither explicitly specified a workspace to extend, and the
``CMAKE_PREFIX_PATH`` environment variable is empty:

.. code-block:: bash

      Extending:                   None

Implicit Chaining via ``CMAKE_PREFIX_PATH`` Environment or Cache Variable
-------------------------------------------------------------------------

In this case, the ``catkin`` command is *implicitly* assuming that you want
to build this workspace against resources which have been built into the
directories listed in your ``CMAKE_PREFIX_PATH`` environment variable. As
such, you can control this value simply by changing this environment
variable.

For example, ROS users who load their system's installed ROS environment by
calling something similar to ``source /opt/ros/hydro/setup.bash`` will
normally see an ``Extending`` value such as:

.. code-block:: bash

      Extending:             [env] /opt/ros/hydro

If you don't want to extend the given workspace, unsetting the
``CMAKE_PREFIX_PATH`` environment variable will change it back to none. You can
also alternatively

Once you have built your workspace once, this ``CMAKE_PREFIX_PATH`` will be
cached by the underlying CMake buildsystem. As such, the ``Extending`` status
will subsequently describe this as the "cached" extension path:

.. code-block:: bash

      Extending:          [cached] /opt/ros/hydro

Once the extension mode is cached like this, you must use ``catkin clean`` to
before changing it to something else.

Explicit Chaining via ``catkin config --extend``
------------------------------------------------

This behaves like the above implicit chaining except it means that this
workspace is *explicitly* extending another workspace and the workspaces
which the other workspace extends, recursively.  This can be set with the
``catkin config --extend`` command. It will override the value of
``CMAKE_PREFIX_PATH`` and persist between builds.

.. code-block:: bash

      Extending:        [explicit] /tmp/path/to/other_ws

