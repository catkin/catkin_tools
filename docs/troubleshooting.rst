Troubleshooting
===============

Configuration Summary Warnings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``catkin`` tool is capable of detecting some issues or inconsistencies with
the build configuration automatically. In these cases, it will often describe
the problem as well as how to resolve it. The ``catkin`` tool will detect the
following issues automatically.

Missing Workspace Components
----------------------------

- Uninitialized workspace (mising ``.catkin_tools`` directory)
- Missing **source space** as specified by the configuration

Inconsistent Environment
------------------------

- The ``CMAKE_PREFIX_PATH`` environment variable is different than the cahced ``CMAKE_PREFIX_PATH``
- The explicitly extended workspace path yeilds a different ``CMAKE_PREFIX_PATH`` than the cached ``CMAKE_PREFIX_PATH``
- The **build space** or **devel space** was built with a different tool such as ``catkin_make`` or ``catkin_make_isolated``
- The **build space** or **devel space** was built in a different isolation mode

Dependency Resolution
^^^^^^^^^^^^^^^^^^^^^

A package can't find certain resources
--------------------------------------

A package doesn't find the right version of a dependency
--------------------------------------------------------
