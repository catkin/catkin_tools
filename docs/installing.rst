Installing ``catkin_tools``
===========================

You can install the ``catkin_tools`` package as a binary through a package manager like ``pip`` or ``apt-get``, or from source.

.. note::

    This project is still in beta and has not been released yet, please install from source.
    In particular, interface and behaviour are still subject to incompatible changes.
    If you rely on a stable environment, please use ``catkin_make`` instead of this tool.

Installing on Ubuntu with apt-get
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First you must have the ROS repositories which contain the ``.deb`` for ``catkin_tools``:

.. code-block:: bash

    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
    $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -

Once you have added that repository, run these commands to install ``catkin_tools``:

.. code-block:: bash

    $ sudo apt-get update
    $ sudo apt-get install python-catkin-tools

Installing on other platforms with pip
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Simply install it with ``pip``:

.. code-block:: bash

    $ sudo pip install -U catkin_tools

Installing from source
^^^^^^^^^^^^^^^^^^^^^^

First clone the source for ``catkin_tools``:

.. code-block:: bash

    $ git clone https://github.com/catkin/catkin_tools.git
    $ cd catkin_tools

Then install with the ``setup.py`` file:

.. code-block:: bash

    $ python setup.py install

Developing
----------

Listed here are some useful tips for developing against ``catkin_tools``.

Install ``catkin_tools`` for developing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To setup ``catkin_tools`` for fast iteration during development, use the ``develop`` verb to ``setup.py``:

.. code-block:: bash

    $ python setup.py develop

Now the commands, like ``catkin``, will be in the system path and the local source files located in the ``catkin_tools`` folder will be on the ``PYTHONPATH``. When you are done with your development, undo this by running this command:

.. code-block:: bash

    $ python setup.py develop -u
