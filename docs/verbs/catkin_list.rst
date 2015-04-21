``catkin list`` -- List Package Info
====================================

The ``list`` verb for the ``catkin`` command is used to find and list
information about catkin packages. By default, it will list the packages in the
workspace containing the current working directoy. It can also be used to list
the packages in any other arbitrary directory.

Checking for Catkin Package Warnings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In addition to the names of the packages in your workspace, running ``catkin
list`` will output any warnings about catkin packages in your workspace. To
suppress these warnings, you can use the ``--quiet`` option.

Using Unformatted Output in Shell Scripts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``catkin list --unformatted`` is useful for automating shell scripts in UNIX
pipe-based programs.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin list [-h] [--deps] [--depends-on [DEPENDS_ON [DEPENDS_ON ...]]]
                       [folders [folders ...]]

    Lists catkin packages in the workspace or other arbitray folders.

    positional arguments:
      folders               Folders in which to find packages. (default: cwd)

    optional arguments:
      -h, --help            show this help message and exit
      --deps, --dependencies
                            List dependencies of each package.
      --depends-on [DEPENDS_ON [DEPENDS_ON ...]]
                            List all packages that depend on supplied argument
                            package(s).
      --quiet               Don't print out detected package warnings.
      --unformatted, -u     Print list without punctuation and additional details.

