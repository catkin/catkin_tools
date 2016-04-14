Shell support in ``catkin`` command
===================================

You can use the ``locate`` verb to locate the shell file for your installation.
When you source the resulting file, you can use ``bash``/``zsh`` shell functions which provide added utility.

.. code-block:: shell

    . `catkin locate --shell-verbs`

Provided verbs are:

- ``catkin cd`` -- Change to package directory in source space.
- ``catkin source`` -- Source the devel space or install space of the containing workspace.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Change to package directory in source space with `cd` verb.

.. code-block:: text

    usage: catkin cd [ARGS...]

    ARGS are any valid catkin locate arguments

The `source` verb sources the devel space or install space of the containing workspace.

.. code-block:: text

    usage: catkin source [-w /path/to/ws]

    Sources setup.sh in the workspace.

    optional arguments:
      -w [/path/to/ws] Source setup.sh from given workspace.
