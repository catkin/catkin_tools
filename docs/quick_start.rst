Quickstart
==========

This chapter gives a high-level overview of how to use ``catkin_tools`` and the ``catkin`` command.
This shows how to use the different command verbs to create and manipulate a workspace.
For a more in-depth explanation of the mechanics of catkin workspaces, see :doc:`Workspace Mechanics <mechanics>`, and for thorough usage details see the individual verb documentation.

TL;DR
^^^^^

The following is an example workflow and sequence of commands using default settings:

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash

.. raw:: html

    <center><script type="text/javascript" src="https://asciinema.org/a/0n70m32pnoc8sexkqp2o5i1lx.js" id="asciicast-0n70m32pnoc8sexkqp2o5i1lx" async></script></center>

Initializing a New Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

While initialization of a workspace can be done automatically with ``catkin build``, it's good practice to initialize a catkin workspace explicitly.
This is done by simply creating a new workspace with an empty **source space** (named ``src`` by default) and calling ``catkin init`` from the workspace root:

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash
   :lines: 1-4

Now the directory ``/tmp/quickstart-init`` has been initialized and ``catkin init`` has printed the standard configuration summary to the console with the default values.
This summary describes the layout of the workspace as well as other important settings which influence build and execution behavior.

Once a workspace has been initialized, the configuration summary can be displayed by calling ``catkin config`` without arguments from anywhere under the root of the workspace.
Doing so will not modify your workspace.
The ``catkin`` command is context-sensitive, so it will determine which workspace contains the current working directory.

An important property which deserves attention is the summary value labeled ``Extending``.
This describes other collections of libraries and packages which will be visible to your workspace.
This is process called "workspace chaining."
The value can come from a few different sources, and can be classified in one of the three following ways:

- No chaining
- Implicit chaining via ``CMAKE_PREFIX_PATH`` environment or cache variable
- Explicit chaining via ``catkin config --extend``

For more information on the configuration summary and workspace chaining, see :doc:`Workspace Configuration <mechanics>`.
For information on manipulating these options, see :doc:`the config verb <verbs/catkin_config>`.

.. note::

    Calling ``catkin init`` "marks" a directory path by creating a hidden
    directory called ``.catkin_tools``. This hidden directory is used to
    designate the parent as the root of a Catkin workspace as well as store
    persistent information about the workspace configuration.

Adding Packages to the Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to build software with Catkin, it needs to be added to the workspace's **source space**.
You can either download some existing packages, or create one or more empty ones.
As shown above, the default path for a Catkin **source space** is `./src` relative to the workspace root.
A standard Catkin package is simply a directory with a ``CMakeLists.txt`` file and a ``package.xml`` file.
For more information on Catkin packages see :doc:`workspace mechanics <mechanics>`.
The shell interaction below shows the creation of four empty packages: ``pkg_a``, ``pkg_b``, ``pkg_c``, and ``pkg_d``:

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash
   :lines: 5-10

After these operations, your workspace's local directory structure would look like the following (to two levels deep):

.. literalinclude:: examples/quickstart_ws/1_prebuild.bash
   :language: bash

.. literalinclude:: examples/quickstart_ws/1_prebuild.out
   :language: text

Now that there are some packages in the workspace, Catkin has something to build.

.. note::

    Catkin utilizes an "out-of-source" and "aggregated" build pattern.
    This means that temporary or final build will products never be placed in a package's source directory (or anywhere in the **source space**.
    Instead all build directories are aggregated in the **build space** and all final build products like executables, libraries, etc., will be put in the **devel space**.

Building the Workspace
^^^^^^^^^^^^^^^^^^^^^^

Since the catkin workspace has already been initialized, you can call ``catkin build`` from any directory contained within it.
If it had not been initialized, then ``catkin build`` would need to be called from the workspace root.
Based on the default configuration, it will locate the packages in the **source space** and build each of them.

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash
   :lines: 11

Calling ``catkin build`` will generate ``build`` and ``devel`` directories (as described in the config summary above) and result in a directory structure like the following (up to one level deep):

.. literalinclude:: examples/quickstart_ws/2_postbuild.bash
   :language: bash

.. literalinclude:: examples/quickstart_ws/2_postbuild.out
   :language: text

Intermediate build products (CMake cache files, Makefiles, object files, etc.) are generated in the ``build`` directory, or **build space** and final build products (libraries, executables, config files) are generated in the ``devel`` directory, or **devel space**.
For more information on building and customizing the build configuration see the :doc:`build verb <verbs/catkin_build>` and :doc:`config verb <verbs/catkin_config>` documentation.

Loading the Workspace Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In order to properly "use" the products of the workspace, its environment needs to be loaded.
Among other environment variables, sourcing a Catkin setup file modifies the ``CMAKE_PREFIX_PATH`` environment variable, which will affect workspace chaining as described in the earlier section.

Setup files are located in one of the **result spaces** generated by your workspace.
Both the **devel space** or the **install space** are valid **result spaces**.
In the default build configuration, only the **devel space** is generated.
You can load the environment for your respective shell like so:

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash
   :lines: 12

At this point you should be able to use products built by any of the packages
in your workspace.

.. note::

    Any time the member packages change in your workspace, you will need to
    re-run the source command.

Loading the environment from a Catkin workspace can set **arbitrarily many** environment variables, depending on which "environment hooks" the member packages define.
As such, it's important to know which workspace environment is loaded in a given shell.

It's not unreasonable to automatically source a given setup file in each shell for convenience, but if you do so, it's good practice to pay attention to the ``Extending`` value in the Catkin config summary.
Any Catkin setup file will modify the ``CMAKE_PREFIX_PATH`` environment variable, and the config summary should catch common inconsistencies in the environment.

Cleaning Workspace Products
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Instead of using dangerous commands like ``rm -rf build devel`` in your workspace when cleaning build products, you can use the ``catkin clean`` command.
Just like the other verbs, ``catkin clean`` is context-aware, so it only needs to be called from a directory under the workspace root.

In order to clean the **build space** and **devel space** for the workspace, you can use the following command:

.. literalinclude:: examples/quickstart_ws/0_quickstart.bash
   :language: bash
   :lines: 13

For more information on less aggressive cleaning options see the :doc:`clean verb <verbs/catkin_clean>` documentation.


