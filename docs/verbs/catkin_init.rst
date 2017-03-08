``catkin init`` -- Initialize a Workspace
=========================================

The ``init`` verb is the simplest way to "initialize" a catkin workspace so that it can be automatically detected automatically by other verbs which need to know the location of the workspace root.

This verb does not store any configuration information, but simply creates the hidden ``.catkin_tools`` directory in the specified workspace.
If you want to initialize a workspace simultaneously with an initial config, see the ``--init`` option for the ``config`` verb.

Catkin workspaces can be initialized anywhere.
The only constraint is that catkin workspaces cannot contain other catkin workspaces.
If you call ``catkin init`` and it reports an error saying that the given directory is already contained in a workspace, you can call ``catkin config`` to determine the root of that workspace.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_init.txt
   :language: text
