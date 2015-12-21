``catkin roslint`` -- Run linter for Packages
=============================================


The ``roslint`` verb for the ``catkin`` command is used to run ``roslint`` make target.
In ROS community, there is a standard `linter <https://github.com/ros/roslint>`_ for C++ and Python.
To run linters for all catkin packages in the workspace, use the following:

.. code-block:: bash

    $ catkin roslint


If you want run ``roslint`` just for on package, do as following:

.. code-block:: bash

    $ catkin roslint package --no-deps
    # Or from a directory within that package:
    $ catkin roslint --no-deps --this
