``catkin list`` -- List Package Info
====================================

The ``list`` verb for the ``catkin`` command is used to find and list information about catkin packages.
By default, it will list the packages in the workspace containing the current working directory in topological order.
It can also be used to list the packages in any other arbitrary directory.

List Package Dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^

``catkin list`` can also show the dependencies of all packages or a subset of the packages when
``catkin list --dependencies`` or ``catkin list --dependencies my_package`` is used.
Recursive dependencies can be listed with ``catkin list --recursive-dependencies``.

Checking for Catkin Package Warnings
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In addition to the names of the packages in your workspace, running ``catkin list`` will output any warnings about catkin packages in your workspace.
To suppress these warnings, you can use the ``--quiet`` option.

Using Unformatted Output in Shell Scripts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

``catkin list --unformatted`` is useful for automating shell scripts in UNIX pipe-based programs.
For ``--dependencies``, valid YAML output is produced.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_list.txt
   :language: text
