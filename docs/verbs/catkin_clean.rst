``catkin clean`` -- Clean Build Products
========================================

The ``clean`` verb makes it easier and safer to clean various products of a catkin workspace.
In addition to removing entire **build**, **devel**, and **install spaces**, it also gives you more fine-grained control over removing just parts of these directories.

The ``clean`` verb is context-aware, but in order to work, it must be given the path to an initialized catkin workspace, or called from a path contained in an initialized catkin workspace.
This is because the paths to the relevant spaces are contained in a workspace's metadata directory.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: cli/catkin_clean.txt
   :language: text
