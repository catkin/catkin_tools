Linked Devel Space
==================

In addition to the ``merged`` and ``isolated`` **devel space** layouts provided by ``catkin_make`` and ``catkin_make_isolated``, respectively, ``catkin_tools`` provides a default ``linked`` layout which enables robust cleaning of individual packages from a workspace.
It does this by building each package into its own hidden FHS tree, and then symbolically linking all products into the unified **devel space** which is specified in the workspace configuration.

When building with a ``linked`` layout, Catkin packages are built into FHS trees stored in the ``.private`` hidden directory at the root of the **devel space**.
Within this directory is a directory for each package in the workspace.

Setup File Generation
^^^^^^^^^^^^^^^^^^^^^

In the ``merged`` layout, every package writes and then over-writes the colliding setup files in the root of the **devel space**.
This leads to race conditions and other problems when trying to parallelize building.
With he ``linked`` layout, however, only one package generates these files, and this is either a built-in "prebuild" package, or if it exists in the workspace, the ``catkin`` CMake package, itself.

.catkin File Generation
^^^^^^^^^^^^^^^^^^^^^^^

When using the ``linked`` layout, ``catkin_tools`` is also responsible for managing the ``.catkin`` file in the root of the **devel space**.
