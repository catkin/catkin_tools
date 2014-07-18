``catkin list`` -- List Package Info
====================================

The ``list`` verb for the ``catkin`` command is used to find and list information about catkin packages.

Full Command-Line Interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: text

    usage: catkin list [-h] [--deps] [--depends-on [DEPENDS_ON [DEPENDS_ON ...]]]
                       [folders [folders ...]]

    Lists catkin packages in a given folder

    positional arguments:
      folders               Folders in which to find packages

    optional arguments:
      -h, --help            show this help message and exit
      --deps, --dependencies
                            list deps of each package
      --depends-on [DEPENDS_ON [DEPENDS_ON ...]]
                            one or more dependencies a package must have to be
                            listed

