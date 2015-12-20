Shell support in ``catkin`` command
===================================

When you source the following file, you can use bash/zsh shell functions which provide added utility.

.. code-block:: shell

    . /opt/ros/indigo/etc/bash_completion.d/catkin_shell_verbs.{bash,zsh}

Provided verbs are:

- ``cd`` -- Change to package directory in source space
- ``source`` -- Source the develspace or installspace of the containing workspace

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Change to package directory in source space with `cd` verb.

.. code-block:: text

    usage: catkin cd [ARGS...]

    ARGS are any valid catkin locate arguments

The `source` verb sources the develspace or installspace of the containing workspace

.. code-block:: text

    usage: catkin source [-w /path/to/ws]

    Sources setup.sh in the workspace.

    optional arguments:
      -w [/path/to/ws] Source setup.sh from given workspace.

