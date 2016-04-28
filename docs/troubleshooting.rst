Troubleshooting
===============

Configuration Summary Warnings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``catkin`` tool is capable of detecting some issues or inconsistencies with the build configuration automatically.
In these cases, it will often describe the problem as well as how to resolve it.
The ``catkin`` tool will detect the following issues automatically.

Missing Workspace Components
----------------------------

- Uninitialized workspace (missing ``.catkin_tools`` directory)
- Missing **source space** as specified by the configuration

Inconsistent Environment
------------------------

- The ``CMAKE_PREFIX_PATH`` environment variable is different than the cached ``CMAKE_PREFIX_PATH``
- The explicitly extended workspace path yields a different ``CMAKE_PREFIX_PATH`` than the cached ``CMAKE_PREFIX_PATH``
- The **build space** or **devel space** was built with a different tool such as ``catkin_make`` or ``catkin_make_isolated``
- The **build space** or **devel space** was built in a different isolation mode

Dependency Resolution
^^^^^^^^^^^^^^^^^^^^^

Packages Are Being Built Out of Order
-------------------------------------

- The ``package.xml`` dependency tags are most likely incorrect.
  Note that   dependencies are only used to order the packages, and there is no warning if   a package can't be found.
- Run ``catkin list --deps /path/to/ws/src`` to list the dependencies of each   package and look for errors.


Incorrect Resolution of Workspace Overlays
------------------------------------------

It's possible for a CMake package to include header directories as ``SYSTEM`` includes pointing to the workspace root include directory (like ``/path/to/ws/devel/include``).
If this happens, CMake will ignore any "normal" includes to that path, and prefer the ``SYSTEM`` include.
This means that ``/path/to/ws/devel/include`` will be searched *after* any other normal includes.
If another package specifies ``/opt/ros/indigo/include`` as a normal include, it will take precedence.

- Minimal example here: https://github.com/jbohren/isystem
- Overview of GCC's system include precedence here: https://gcc.gnu.org/onlinedocs/cpp/System-Headers.html

As a workaround, you can force CMake to ignore all specified root include directories, and rely on CPATH for header resolution in these paths:

.. code-block:: bash

    catkin config -a --cmake-args -DCMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES="/opt/ros/indigo/include"

This is actually a bug in CMake and has been reported here: https://cmake.org/Bug/view.php?id=15970

Command-Line Output
^^^^^^^^^^^^^^^^^^^

Build Errors or Warnings Not Printing
-------------------------------------

By default, ``catkin build`` will only print error and warning messages which are written to ``stderr``.
Even if a package fails, these messages will not be shown if they are only written to ``stdout``.

If your build employs tools which print errors to ``stdout`` instead of ``stderr``, then you should use the ``--verbose-errors`` option.
This will cause all captured output from a subprocess to be printed in the event of an error:

.. code-block:: bash

    catkin build --verbose-errors

If you always want to use this option, you can add it as a verb alias.
See :doc:`Verb Aliasing <advanced/verb_customization>` for more info on adding aliases.

The following programs are known to redirect error messages to ``stdout``:

- ``colorgcc`` - https://github.com/johannes/colorgcc



Migration Problems
^^^^^^^^^^^^^^^^^^

For troubleshooting problems when migrating from ``catkin_make`` or ``catkin_make_isolated``, see :ref:`migration-troubleshooting`.
